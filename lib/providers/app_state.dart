import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'dart:convert';
import 'dart:async';
import '../models/models.dart';

class AppState extends ChangeNotifier {
  static const List<String> cocoClasses = [
    'Person', 'Bicycle', 'Car', 'Motorcycle', 'Airplane', 'Bus', 'Train', 'Truck', 'Boat', 'Traffic Light',
    'Fire Hydrant', 'Stop Sign', 'Parking Meter', 'Bench', 'Bird', 'Cat', 'Dog', 'Horse', 'Sheep', 'Cow',
    'Elephant', 'Bear', 'Zebra', 'Giraffe', 'Backpack', 'Umbrella', 'Handbag', 'Tie', 'Suitcase', 'Frisbee',
    'Skis', 'Snowboard', 'Sports Ball', 'Kite', 'Baseball Bat', 'Baseball Glove', 'Skateboard', 'Surfboard',
    'Tennis Racket', 'Bottle', 'Wine Glass', 'Cup', 'Fork', 'Knife', 'Spoon', 'Bowl', 'Banana', 'Apple',
    'Sandwich', 'Orange', 'Broccoli', 'Carrot', 'Hot Dog', 'Pizza', 'Donut', 'Cake', 'Chair', 'Couch',
    'Potted Plant', 'Bed', 'Dining Table', 'Toilet', 'TV', 'Laptop', 'Mouse', 'Remote', 'Keyboard', 'Cell Phone',
    'Microwave', 'Oven', 'Toaster', 'Sink', 'Refrigerator', 'Book', 'Clock', 'Vase', 'Scissors', 'Teddy Bear',
    'Hair Drier', 'Toothbrush'
  ];

  static const MethodChannel _commandChannel = MethodChannel('com.hsv2/command');
  static const EventChannel _telemetryChannel = EventChannel('com.hsv2/telemetry');
  static const EventChannel _usbStatusChannel = EventChannel('com.hsv2/usb_status');

  int? _textureId;
  int? get textureId => _textureId;

  int? _depthTextureId;
  int? get depthTextureId => _depthTextureId;

  // Telemetry
  TelemetryData _telemetry = TelemetryData();
  TelemetryData get telemetry => _telemetry;

  // Pipeline Status
  bool _isPipelineRunning = false;
  bool get isPipelineRunning => _isPipelineRunning;

  // Camera Connection Status
  String _usbStatus = kIsWeb ? "CONNECTED" : "DISCONNECTED";
  String get usbStatus => _usbStatus;
  bool get isCameraConnected => _usbStatus == "CONNECTED";

  // Pipeline problem to show in the UI (start failure or no frames), or null.
  String? _streamError;
  String? get streamError => _streamError;
  // The camera was unplugged while running (cleared on the next START).
  bool _cameraLost = false;
  bool get cameraLost => _cameraLost;

  // Stage Config
  StageConfig _stageConfig = StageConfig(currentStage: PipelineStage.stage1);
  StageConfig get stageConfig => _stageConfig;

  // Detections
  List<DetectionResult> _detections = [];
  List<DetectionResult> get detections => _detections;

  // Mock Timer for Web
  Timer? _mockWebTimer;

  // Telemetry: subscribed once. Native only sends while the camera streams,
  // so telemetry arriving means "running" (keeps the START/STOP button in
  // sync after the activity is recreated, e.g. coming back from Settings),
  // and 2 s without telemetry means it stopped.
  StreamSubscription? _telemetrySub;
  DateTime _lastTelemetry = DateTime.fromMillisecondsSinceEpoch(0);
  Timer? _streamWatch;
  Timer? _voiceWatch;

  AppState() {
    if (!kIsWeb) {
      _startListeningToUsbStatus();
      _startListeningToTelemetry();
      _commandChannel.invokeMethod('getLiveDescribe').then((r) {
        if (r is bool) { liveDescribe = r; notifyListeners(); }
      }).catchError((_) {});
      _commandChannel.invokeMethod('getVoice').then((r) {
        if (r is String) { _applyVoiceStatus(jsonDecode(r)); notifyListeners(); }
      }).catchError((_) {});
      _streamWatch = Timer.periodic(const Duration(seconds: 1), (_) {
        if (_isPipelineRunning && !_stopRequested &&
            DateTime.now().difference(_lastTelemetry) > const Duration(seconds: 2) &&
            DateTime.now().difference(_startedAt) > const Duration(seconds: 8)) {
          _isPipelineRunning = false;
          notifyListeners();
        }
      });

      // Auto-start Voice Assistant immediately on app launch
      Future.delayed(const Duration(milliseconds: 600), () {
        startVoiceAssistant(greet: true);
      });

      // Periodic synchronization for voice assistant state & transcript
      _voiceWatch = Timer.periodic(const Duration(milliseconds: 400), (_) {
        _commandChannel.invokeMethod('getVoiceAssistantStatus').then((r) {
          if (r is String) {
            _applyVoiceAssistantStatus(jsonDecode(r));
          }
        }).catchError((_) {});
      });
    }
  }

  @override
  void dispose() {
    _voiceWatch?.cancel();
    _streamWatch?.cancel();
    _telemetrySub?.cancel();
    super.dispose();
  }

  bool _stopRequested = false;
  DateTime _startedAt = DateTime.fromMillisecondsSinceEpoch(0);

  Future<void> initNativeTexture() async {
    if (_textureId != null) return;
    try {
      final int id = await _commandChannel.invokeMethod('createTexture');
      _textureId = id;
      notifyListeners();
    } catch (e) {
      debugPrint('Failed to create native texture: $e');
    }
  }

  Future<void> initDepthTexture() async {
    if (_depthTextureId != null) return;
    try {
      final int id = await _commandChannel.invokeMethod('createDepthTexture');
      _depthTextureId = id;
      notifyListeners();
    } catch (e) {
      debugPrint('Failed to create native depth texture: $e');
    }
  }

  void _startListeningToUsbStatus() {
    _usbStatusChannel.receiveBroadcastStream().listen((dynamic event) {
      _usbStatus = event.toString();
      // Native side stops the pipeline on detach; mirror that here.
      if (_usbStatus != "CONNECTED" && _isPipelineRunning) {
        _isPipelineRunning = false;
        _detections = [];
        _streamError = "Camera disconnected";
        _cameraLost = true;   // sensor health: shown as STOP until START again
      }
      notifyListeners();
    }, onError: (dynamic error) {
      debugPrint("USB Status error: $error");
    });
  }

  void setStageConfig(StageConfig config) async {
    _stageConfig = config;
    if (kIsWeb) {
      notifyListeners();
      return;
    }
    
    try {
      await _commandChannel.invokeMethod('setStage', {'stage': config.currentStage.index + 1});
    } on PlatformException catch (e) {
      debugPrint("Failed to set stage: '${e.message}'.");
    }
    notifyListeners();
  }

  /// Camera view: 0 = camera only, 1 = camera + segmentation overlay,
  /// 2 = segmentation only, 3 = 3D map view.
  int viewMode = 1;
  static const viewModeLabels = ['CAM', 'MIX', 'SEG', '3D'];

  /// Walk recorder (REC button).
  bool recording = false;

  Future<void> toggleRecording() async {
    recording = !recording;
    notifyListeners();
    if (kIsWeb) return;
    try {
      await _commandChannel.invokeMethod(recording ? 'startRecording' : 'stopRecording');
    } on PlatformException catch (e) {
      debugPrint("Recording toggle failed: '${e.message}'.");
    }
  }

  Future<void> cycleViewMode() async {
    viewMode = (viewMode + 1) % 4;
    notifyListeners();
    if (kIsWeb) return;
    try {
      await _commandChannel.invokeMethod('setViewMode', {'mode': viewMode});
    } on PlatformException catch (e) {
      debugPrint("Failed to set view mode: '${e.message}'.");
    }
  }

  // Phase 6 voice guidance (spoken by MainActivity / Sailens output).
  bool voiceEnabled = true;
  String voiceLang = 'en';          // chosen: 'en' | 'ta'
  String voiceSpoken = 'en';        // actually spoken (Tamil needs a Tamil TTS voice)
  bool voiceReady = false;
  bool tamilVoiceInstalled = false;

  void _applyVoiceStatus(dynamic v) {
    if (v is! Map) return;
    voiceEnabled = v['enabled'] == true;
    voiceLang = v['lang'] as String? ?? 'en';
    voiceSpoken = v['spoken'] as String? ?? 'en';
    voiceReady = v['ready'] == true;
    tamilVoiceInstalled = v['tamil'] == true;
  }

  Future<void> setVoice({bool? enabled, String? lang}) async {
    voiceEnabled = enabled ?? voiceEnabled;
    voiceLang = lang ?? voiceLang;
    notifyListeners();
    if (kIsWeb) return;
    try {
      final r = await _commandChannel.invokeMethod('setVoice', {'enabled': voiceEnabled, 'lang': voiceLang});
      if (r is String) _applyVoiceStatus(jsonDecode(r));
      notifyListeners();
    } on PlatformException catch (e) {
      debugPrint("Failed to set voice: '${e.message}'.");
    }
  }

  // Phase 7 validation: one scenario at a time, logged to a CSV on the phone.
  static const List<(String, String, String)> scenarios = [
    ('S1', 'Wall approach', 'Start 3 m from a wall. Walk slowly straight at it. Stop when told, stand 5 s, step back.'),
    ('S2', 'Clear corridor', 'Walk 10 m along a clear corridor at normal pace.'),
    ('S3', 'Doorway', 'Start 3 m before an open door and walk through it.'),
    ('S4', 'Corner turn', 'Walk to a corridor corner and follow it round.'),
    ('S5', 'Person approaching', 'Stand still. A helper walks towards you from 4 m.'),
    ('S6', 'Step down / stairs', 'With a helper: walk slowly towards the top of a step or stairs.'),
    ('S7', 'Camera covered', 'While walking, cover the camera with your hand for 3 s, then uncover.'),
    ('S8', 'Standing still', 'Stand still for 30 s facing an open space.'),
    ('S9', 'Chair in path', 'Put a chair in the middle of a corridor. Walk towards it.'),
  ];
  String? scenarioId;
  String scenarioLabel = '';

  Future<void> startScenario(String id, String label) async {
    if (kIsWeb) return;
    try {
      await _commandChannel.invokeMethod('startScenario', {'id': id, 'label': label});
      scenarioId = id;
      scenarioLabel = label;
      if (!_isPipelineRunning) togglePipeline();
      notifyListeners();
    } on PlatformException catch (e) {
      debugPrint("Failed to start scenario: '${e.message}'.");
    }
  }

  Future<void> stopScenario() async {
    if (kIsWeb) return;
    try {
      await _commandChannel.invokeMethod('stopScenario');
    } on PlatformException catch (e) {
      debugPrint("Failed to stop scenario: '${e.message}'.");
    }
    scenarioId = null;
    notifyListeners();
  }

  // "What's ahead?" (Gemma 4 E2B on the phone): last answer, shown as a caption.
  // liveDescribe: Gemma also comments by itself during navigation (events).
  bool liveDescribe = false;

  Future<void> setLiveDescribe(bool on) async {
    liveDescribe = on;
    notifyListeners();
    if (kIsWeb) return;
    try {
      final r = await _commandChannel.invokeMethod('setLiveDescribe', {'on': on});
      if (r is bool) liveDescribe = r;
    } on PlatformException catch (e) {
      debugPrint("Failed to set live describe: '${e.message}'.");
    }
    notifyListeners();
  }

  String sceneText = '';
  bool sceneBusy = false;

  Future<void> describeScene() async {
    if (kIsWeb || sceneBusy) return;
    sceneBusy = true;
    sceneText = 'Looking…';
    notifyListeners();
    try {
      final r = await _commandChannel.invokeMethod('describeScene');
      sceneText = r is String ? r : '';
    } on PlatformException catch (e) {
      sceneText = 'Description failed: ${e.message ?? e.code}';
    }
    sceneBusy = false;
    notifyListeners();
  }

  // Voice Assistant (Argus Offline Assistant)
  String _voiceAssistantState = 'IDLE';
  String get voiceAssistantState => _voiceAssistantState;
  bool get isVoiceListening => _voiceAssistantState == 'LISTENING';
  bool get isVoiceSpeaking => _voiceAssistantState == 'SPEAKING';
  String _voiceAssistantTranscript = '';
  String get voiceAssistantTranscript => _voiceAssistantTranscript;

  Future<void> startVoiceAssistant({bool greet = false}) async {
    if (kIsWeb) return;
    try {
      final r = await _commandChannel.invokeMethod('startVoiceAssistant', {'greet': greet});
      _voiceAssistantState = (r is String && r.isNotEmpty) ? r : (greet ? 'SPEAKING' : 'LISTENING');
      notifyListeners();
    } on PlatformException catch (e) {
      debugPrint("startVoiceAssistant failed: ${e.message}");
    }
  }

  Future<void> stopVoiceAssistant() async {
    if (kIsWeb) return;
    try {
      await _commandChannel.invokeMethod('stopVoiceAssistant');
    } on PlatformException catch (e) {
      debugPrint("stopVoiceAssistant failed: ${e.message}");
    }
  }

  Future<void> toggleVoiceAssistant() async {
    if (kIsWeb) return;
    try {
      final r = await _commandChannel.invokeMethod('toggleVoiceAssistant');
      if (r is String) _voiceAssistantState = r;
      notifyListeners();
    } on PlatformException catch (e) {
      debugPrint("toggleVoiceAssistant failed: ${e.message}");
    }
  }

  Future<String?> triggerVoiceCommand(String text) async {
    if (kIsWeb) return null;
    try {
      final r = await _commandChannel.invokeMethod('triggerVoiceCommand', {'text': text});
      return r is String ? r : null;
    } on PlatformException catch (e) {
      debugPrint("triggerVoiceCommand failed: ${e.message}");
      return null;
    }
  }

  void _applyVoiceAssistantStatus(dynamic raw) {
    if (raw is Map) {
      final state = raw['state'] as String?;
      if (state != null && state != _voiceAssistantState) {
        _voiceAssistantState = state;
      }
      final transcript = raw['transcript'] as String?;
      if (transcript != null) {
        _voiceAssistantTranscript = transcript;
      }
      notifyListeners();
    }
  }

  /// Use the current orientation as level (phone IMU mount offset).
  Future<void> zeroImu() async {
    if (kIsWeb) return;
    try {
      await _commandChannel.invokeMethod('zeroImu');
    } on PlatformException catch (e) {
      debugPrint("Failed to zero IMU: '${e.message}'.");
    }
  }

  void togglePipeline() async {
    if (_isPipelineRunning) {
      _stopRequested = true;
      if (kIsWeb) {
        _stopWebMock();
      } else {
        try {
          debugPrint('HSV2: STOP pressed (button)');
          await _commandChannel.invokeMethod('stopPipeline', {'source': 'button'});
        } on PlatformException catch (e) {
          debugPrint("Failed to stop pipeline: '${e.message}'.");
        }
      }
      _isPipelineRunning = false;
      _stopRequested = false;
      scenarioId = null;   // STOP also ends a running validation test (saved by MainActivity)
    } else {
      _isPipelineRunning = true;
      _startedAt = DateTime.now();   // camera start can take a few seconds
      _streamError = null;
      _cameraLost = false;
      if (kIsWeb) {
        _startWebMock();
      } else {
        try {
          await _commandChannel.invokeMethod('startPipeline');
        } on PlatformException catch (e) {
          debugPrint("Failed to start pipeline: '${e.message}'.");
          _isPipelineRunning = false;
          _streamError = "Start failed: ${e.message ?? e.code}";
        }
      }
    }
    notifyListeners();
  }

  void _startWebMock() {
    _mockWebTimer = Timer.periodic(const Duration(milliseconds: 100), (timer) {
      if (!_isPipelineRunning) return;
      _telemetry = TelemetryData(
        fps: 15.0 + (DateTime.now().millisecondsSinceEpoch % 5) / 10,
        p50LatencyMs: 30.0,
        p95LatencyMs: 45.0,
        p99LatencyMs: 60.0,
        cpuUsage: 25.0,
        gpuUsage: 10.0,
        npuUsage: 40.0,
        thermalState: 35.0,
        batteryLevel: 98.0,
        droppedFrames: 0,
        usbTransportLatencyMs: 2.0,
        centerDistanceMeters: 1.25,
      );

      // Removed static mock detection per user request
      _detections = [];
      notifyListeners();
    });
  }

  void _stopWebMock() {
    _mockWebTimer?.cancel();
    _mockWebTimer = null;
    _detections = [];
  }

  void _startListeningToTelemetry() {
    if (_telemetrySub != null) return;
    _telemetrySub = _telemetryChannel.receiveBroadcastStream().listen((dynamic event) {
      _lastTelemetry = DateTime.now();
      if (_stopRequested) return;
      if (!_isPipelineRunning) {
        _isPipelineRunning = true;   // native is streaming: resync the button
      }
      try {
        final Map<String, dynamic> data = jsonDecode(event as String);
        if (data['status'] == 'no_frames') {
          _streamError = "No frames from camera";
          _detections = [];
          // Sensor health: camera lost -> STOP (no walking advice).
          final g = data['guide'] as Map?;
          _telemetry = TelemetryData(
            imu: _telemetry.imu,
            alert: 'stop',
            alertReason: 'sensor',
            guideCode: g?['code'] as String? ?? 'sensor_fault',
            guideText: g?['text'] as String? ?? 'STOP! Camera lost.',
            guideSeq: -1,
          );
          notifyListeners();
          return;
        }
        _streamError = null;
        _applyVoiceStatus(data['voice']);
        _applyVoiceAssistantStatus(data['voiceAssistant']);
        _telemetry = TelemetryData(
          fps: (data['fps'] as num?)?.toDouble() ?? 0.0,
          p50LatencyMs: (data['p50'] as num?)?.toDouble() ?? 0.0,
          p95LatencyMs: (data['p95'] as num?)?.toDouble() ?? 0.0,
          p99LatencyMs: (data['p99'] as num?)?.toDouble() ?? 0.0,
          cpuUsage: (data['cpu'] as num?)?.toDouble() ?? 0.0,
          gpuUsage: (data['gpu'] as num?)?.toDouble() ?? 0.0,
          npuUsage: (data['npu'] as num?)?.toDouble() ?? 0.0,
          thermalState: (data['thermal'] as num?)?.toDouble() ?? 0.0,
          batteryLevel: (data['battery'] as num?)?.toDouble() ?? 100.0,
          centerDistanceMeters: (data['centerDistance'] as num?)?.toDouble() ?? 0.0,
          droppedFrames: (data['droppedFrames'] as num?)?.toInt() ?? 0,
          alert: data['alert'] as String? ?? 'none',
          alertReason: data['alertReason'] as String? ?? 'none',
          alertDist: (data['alertDist'] as num?)?.toDouble() ?? -1.0,
          alertTtc: (data['alertTtc'] as num?)?.toDouble() ?? -1.0,
          planOk: (data['plan'] as Map?)?['ok'] == true,
          planBlocked: (data['plan'] as Map?)?['blocked'] == true,
          planTurn: ((data['plan'] as Map?)?['turn'] as num?)?.toDouble() ?? 0.0,
          planLength: ((data['plan'] as Map?)?['length'] as num?)?.toDouble() ?? 0.0,
          guideCode: (data['guide'] as Map?)?['code'] as String? ?? 'none',
          // The live situation sentence (same as the voice) when there is one.
          guideText: (data['guide'] as Map?)?['sentence'] as String? ??
              (data['guide'] as Map?)?['text'] as String? ?? '',
          guideSeq: ((data['guide'] as Map?)?['seq'] as num?)?.toInt() ?? 0,
          aheadSrc: (data['validator'] as Map?)?['aheadSrc'] as String? ?? '',
          imu: data['imu'] is Map<String, dynamic>
              ? ImuData.fromJson(data['imu'] as Map<String, dynamic>)
              : const ImuData(),
        );

        if (data.containsKey('detections')) {
          final List<dynamic> detList = data['detections'];
          _detections = detList.map((d) {
            final bboxList = (d['bbox'] as List<dynamic>?)?.map((e) => (e as num).toDouble()).toList() ?? [0.0, 0.0, 0.0, 0.0];
            final int classId = d['class'] as int? ?? -1;
            final String className = (classId >= 0 && classId < cocoClasses.length) 
                ? cocoClasses[classId] 
                : 'Object $classId';
                
            return DetectionResult(
              objectName: className,
              confidence: (d['confidence'] as num?)?.toDouble() ?? 0.0,
              distanceMeters: (d['distanceMeters'] as num?)?.toDouble() ?? 0.0,
              directionClock: '',
              boundingBox: bboxList,
              masked: d['masked'] as bool? ?? false,
              trackId: (d['track'] as num?)?.toInt() ?? -1,
              speed: (d['speed'] as num?)?.toDouble() ?? 0.0,
              approaching: d['approaching'] as bool? ?? false,
              ttc: (d['ttc'] as num?)?.toDouble() ?? -1.0,
            );
          }).toList();
        } else {
          _detections = [];
        }
        notifyListeners();
      } catch (e) {
        debugPrint("Error parsing telemetry: $e");
      }
    }, onError: (dynamic error) {
      debugPrint("Telemetry error: $error");
    });
  }
}
