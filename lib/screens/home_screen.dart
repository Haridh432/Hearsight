import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';
import 'package:provider/provider.dart';
import 'package:camera/camera.dart';
import '../providers/app_state.dart';
import '../models/models.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  CameraController? _cameraController;
  bool _isCameraInitialized = false;

  @override
  void initState() {
    super.initState();
    if (kIsWeb) {
      _initMockWebCamera();
    } else {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        Provider.of<AppState>(context, listen: false).initNativeTexture();
        Provider.of<AppState>(context, listen: false).initDepthTexture();
      });
    }
  }

  Future<void> _initMockWebCamera() async {
    try {
      final cameras = await availableCameras();
      if (cameras.isNotEmpty) {
        _cameraController = CameraController(
          cameras.first,
          ResolutionPreset.medium,
          enableAudio: false,
        );
        await _cameraController!.initialize();
        if (mounted) {
          setState(() {
            _isCameraInitialized = true;
          });
        }
      } else {
        debugPrint("No cameras found by the browser.");
      }
    } catch (e) {
      debugPrint("Error initializing web camera: $e");
    }
  }

  @override
  void dispose() {
    _cameraController?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final state = context.watch<AppState>();
    
    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(
        children: [
          // 1. Camera Feeds (RGB and Depth Side-by-Side)
          Positioned.fill(
            child: kIsWeb && _isCameraInitialized && _cameraController != null
                ? SizedBox(
                    width: double.infinity,
                    height: double.infinity,
                    child: FittedBox(
                      fit: BoxFit.cover,
                      child: SizedBox(
                        width: _cameraController!.value.previewSize?.height ?? 1,
                        height: _cameraController!.value.previewSize?.width ?? 1,
                        child: CameraPreview(_cameraController!),
                      ),
                    ),
                  )
                : (!kIsWeb && state.textureId != null && state.isCameraConnected)
                    ? Stack(
                        fit: StackFit.expand,
                        children: [
                          Column(
                            children: [
                              // Top Half: RGB Feed
                              Expanded(
                                child: ClipRect(
                                child: LayoutBuilder(
                                  builder: (context, constraints) => Stack(
                                  fit: StackFit.expand,
                                  children: [
                                    FittedBox(
                                      fit: BoxFit.cover,
                                      clipBehavior: Clip.hardEdge,
                                      child: SizedBox(
                                        width: 640,
                                        height: 480,
                                        child: state.textureId != null ? Texture(textureId: state.textureId!) : const SizedBox(),
                                      ),
                                    ),
                                    // No camera labels over the 3D map view.
                                    if (state.viewMode != 3) ..._detectionLabels(
                                      context,
                                      state.detections,
                                      constraints.biggest,
                                      MediaQuery.of(context).padding.top,
                                    ),
                                    // Moving-object alert (time-to-collision), below the status bar.
                                    if (state.telemetry.alert != 'none')
                                      Positioned(
                                        top: MediaQuery.of(context).padding.top + 4,
                                        left: 0,
                                        right: 0,
                                        child: Center(child: _motionAlertBanner(state.telemetry)),
                                      ),
                                    // 3D view: colour legend (top-left, under the IMU line).
                                    if (state.viewMode == 3)
                                      Positioned(top: 150, left: 8, child: _legend3d()),
                                    // Phase 7: running validation test.
                                    if (state.scenarioId != null)
                                      Positioned(
                                        top: MediaQuery.of(context).padding.top + 52,
                                        left: 0,
                                        right: 0,
                                        child: Center(
                                          child: GestureDetector(
                                            onTap: () => context.read<AppState>().stopScenario(),
                                            child: Semantics(
                                              button: true,
                                              label: 'Test ${state.scenarioId} ${state.scenarioLabel} running. Double tap to finish',
                                              excludeSemantics: true,
                                              child: Container(
                                                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                                                decoration: BoxDecoration(color: Colors.amber.shade700, borderRadius: BorderRadius.circular(12)),
                                                child: Text('TEST ${state.scenarioId} · ${state.scenarioLabel} — tap to finish',
                                                    style: const TextStyle(color: Colors.black, fontWeight: FontWeight.bold)),
                                              ),
                                            ),
                                          ),
                                        ),
                                      ),
                                    // Phase 5 guidance instruction (every view).
                                    if (state.isPipelineRunning)
                                      Positioned(
                                        bottom: 8,
                                        left: 0,
                                        right: 0,
                                        child: Center(child: _planChip(state.telemetry)),
                                      ),
                                    // IMU panel: compact, top-left under the alert banner
                                    // (the top of the view is ceiling; the bottom shows
                                    // the floor, steps and low obstacles).
                                    Positioned(
                                      top: 120,
                                      left: 8,
                                      // Tap to zero (level the mount offset).
                                      child: GestureDetector(
                                        onTap: () => context.read<AppState>().zeroImu(),
                                        child: _imuPanel(state.telemetry.imu),
                                      ),
                                    ),
                                  ],
                                ),
                                ),
                                ),
                              ),
                              // Bottom Half: Depth Feed
                              Expanded(
                                child: ClipRect(
                                child: FittedBox(
                                  fit: BoxFit.cover,
                                  clipBehavior: Clip.hardEdge,
                                  child: SizedBox(
                                    width: 640,
                                    height: 480,
                                    child: state.depthTextureId != null ? Texture(textureId: state.depthTextureId!) : const SizedBox(),
                                  ),
                                ),
                                ),
                              ),
                            ],
                          ),
                        ],
                      )
                    : state.cameraLost
                        // Sensor health: the camera was unplugged while walking.
                        ? Container(
                            color: Colors.red.shade800,
                            child: const Center(
                              child: Column(mainAxisSize: MainAxisSize.min, children: [
                                Icon(Icons.videocam_off, color: Colors.white, size: 72),
                                SizedBox(height: 12),
                                Text('STOP! Camera lost.',
                                    style: TextStyle(color: Colors.white, fontSize: 32, fontWeight: FontWeight.bold)),
                                SizedBox(height: 8),
                                Text('Reconnect the camera, then press START',
                                    style: TextStyle(color: Colors.white70, fontSize: 16)),
                              ]),
                            ),
                          )
                        : Container(
                            color: Colors.black87,
                            child: const Center(
                              child: Text(
                                "WAITING FOR NATIVE REALSENSE FEED",
                                style: TextStyle(color: Colors.white54, letterSpacing: 2),
                              ),
                            ),
                          ),
          ),
          
          // 2. Removed static detections mapped in position (replaced by overlay in CustomPaint)

          // 3. Floating Control Panel Overlay
          Positioned(
            left: 16,
            bottom: 16,
            right: 16,
            child: Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: Colors.black.withValues(alpha: 0.7),
                borderRadius: BorderRadius.circular(16),
                border: Border.all(color: Colors.white24),
              ),
              child: Column(mainAxisSize: MainAxisSize.min, children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  // Status & Metrics Column
                  Expanded(child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          Icon(
                            Icons.usb,
                            color: _getStatusColor(state.usbStatus),
                            size: 16,
                          ),
                          const SizedBox(width: 4),
                          Text(
                            state.usbStatus,
                            style: TextStyle(color: _getStatusColor(state.usbStatus), fontWeight: FontWeight.bold),
                          ),
                        ],
                      ),
                      const SizedBox(height: 8),
                      Text('FPS ${state.telemetry.fps.toStringAsFixed(0)} · ${state.telemetry.p95LatencyMs.toStringAsFixed(0)} ms${state.recording ? ' · REC' : ''}',
                          style: TextStyle(color: state.recording ? Colors.redAccent : Colors.white70)),
                      if (state.sceneText.isNotEmpty)
                        Padding(
                          padding: const EdgeInsets.only(top: 4),
                          child: Text(state.sceneText,
                              maxLines: 3,
                              style: const TextStyle(color: Colors.white, fontSize: 14, fontStyle: FontStyle.italic)),
                        ),
                      if (state.voiceAssistantTranscript.isNotEmpty || state.isVoiceListening)
                        Padding(
                          padding: const EdgeInsets.only(top: 4),
                          child: Row(
                            mainAxisSize: MainAxisSize.min,
                            children: [
                              Icon(
                                state.isVoiceListening ? Icons.mic : Icons.record_voice_over,
                                color: state.isVoiceListening ? Colors.redAccent : Colors.tealAccent,
                                size: 16,
                              ),
                              const SizedBox(width: 6),
                              Flexible(
                                child: Text(
                                  state.isVoiceListening && state.voiceAssistantTranscript.isEmpty
                                      ? 'Listening for voice command…'
                                      : 'Voice: "${state.voiceAssistantTranscript}"',
                                  maxLines: 2,
                                  overflow: TextOverflow.ellipsis,
                                  style: TextStyle(
                                    color: state.isVoiceListening ? Colors.redAccent : Colors.tealAccent,
                                    fontSize: 13,
                                    fontWeight: FontWeight.w600,
                                  ),
                                ),
                              ),
                            ],
                          ),
                        ),
                      if (state.streamError != null)
                        Padding(
                          padding: const EdgeInsets.only(top: 4),
                          child: Text(state.streamError!, style: const TextStyle(color: Colors.redAccent, fontWeight: FontWeight.bold)),
                        ),
                    ],
                  )),
                  
                  // Removed Stage Config Dropdown per user request
                  
                  // View switch (camera / overlay / segmentation) + Start / Stop
                  Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                  IconButton(
                    tooltip: state.recording ? 'Stop recording' : 'Record walk',
                    onPressed: state.isPipelineRunning ? () => context.read<AppState>().toggleRecording() : null,
                    icon: Icon(state.recording ? Icons.stop_circle : Icons.fiber_manual_record,
                        color: state.recording ? Colors.redAccent : Colors.white70),
                  ),
                  OutlinedButton.icon(
                    onPressed: () => context.read<AppState>().cycleViewMode(),
                    icon: const Icon(Icons.layers),
                    label: Text(AppState.viewModeLabels[state.viewMode]),
                    style: OutlinedButton.styleFrom(
                      padding: const EdgeInsets.symmetric(horizontal: 10),
                      foregroundColor: Colors.white,
                      side: const BorderSide(color: Colors.white54),
                    ),
                  ),
                  const SizedBox(width: 8),
                  Semantics(
                    button: true,
                    enabled: state.isCameraConnected,
                    label: state.isPipelineRunning
                        ? 'Stop navigation'
                        : (state.isCameraConnected ? 'Start navigation' : 'Start navigation, camera not connected'),
                    excludeSemantics: true,
                    onTap: state.isCameraConnected ? () => context.read<AppState>().togglePipeline() : null,
                    child: ElevatedButton.icon(
                    onPressed: state.isCameraConnected ? () => context.read<AppState>().togglePipeline() : null,
                    icon: Icon(state.isPipelineRunning ? Icons.stop : Icons.play_arrow, size: 30),
                    label: Text(state.isPipelineRunning ? 'STOP' : 'START',
                        style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
                    style: ElevatedButton.styleFrom(
                      minimumSize: const Size(120, 60),   // large touch target
                      padding: const EdgeInsets.symmetric(horizontal: 12),
                      backgroundColor: state.isPipelineRunning ? Colors.red : Colors.teal,
                      foregroundColor: Colors.white,
                    ),
                  ),
                  ),
                    ],
                  ),
                ],
              ),
              // Voice, language and validation-test buttons: their own full-width row, so
              // they never overlap the START / STOP button.
              // Phase 6: voice on/off and language (English / Tamil).
              Row(mainAxisSize: MainAxisSize.min, children: [
                IconButton(
                  tooltip: state.voiceEnabled ? 'Voice guidance on. Double tap to turn off' : 'Voice guidance off. Double tap to turn on',
                  visualDensity: VisualDensity.compact,
                  onPressed: () => context.read<AppState>().setVoice(enabled: !state.voiceEnabled),
                  icon: Icon(state.voiceEnabled ? Icons.volume_up : Icons.volume_off,
                      color: state.voiceEnabled ? Colors.white : Colors.white38),
                ),
                Semantics(
                  button: true,
                  label: switch (state.voiceLang) {
                    'ta' => 'Voice language Tamil. Double tap for Tamil and English',
                    'both' => 'Voice language Tamil and English. Double tap for English',
                    _ => 'Voice language English. Double tap for Tamil',
                  },
                  excludeSemantics: true,
                  onTap: () => context.read<AppState>()
                      .setVoice(lang: switch (state.voiceLang) { 'en' => 'ta', 'ta' => 'both', _ => 'en' }),
                  child: TextButton(
                  onPressed: () => context.read<AppState>()
                      .setVoice(lang: switch (state.voiceLang) { 'en' => 'ta', 'ta' => 'both', _ => 'en' }),
                  child: Text(switch (state.voiceLang) { 'ta' => 'தமிழ்', 'both' => 'தமிழ் + English', _ => 'English' },
                      style: const TextStyle(color: Colors.white, fontWeight: FontWeight.bold)),
                ),
                ),
                IconButton(
                  tooltip: state.liveDescribe
                      ? 'Live descriptions on. Double tap to turn off'
                      : 'Live descriptions off. Double tap to turn on',
                  visualDensity: VisualDensity.compact,
                  onPressed: () => context.read<AppState>().setLiveDescribe(!state.liveDescribe),
                  icon: Icon(Icons.auto_awesome,
                      color: state.liveDescribe ? Colors.amber : Colors.white38),
                ),
                IconButton(
                  tooltip: "What's ahead? Describe the scene",
                  visualDensity: VisualDensity.compact,
                  onPressed: state.isPipelineRunning && !state.sceneBusy
                      ? () => context.read<AppState>().describeScene()
                      : null,
                  icon: Icon(state.sceneBusy ? Icons.hourglass_top : Icons.record_voice_over,
                      color: state.isPipelineRunning ? Colors.white : Colors.white38),
                ),
                IconButton(
                  tooltip: state.isVoiceListening
                      ? 'Listening to voice command… Tap to cancel'
                      : 'Voice Assistant. Tap to speak command',
                  visualDensity: VisualDensity.compact,
                  onPressed: () => context.read<AppState>().toggleVoiceAssistant(),
                  icon: Icon(
                    state.isVoiceListening ? Icons.mic : Icons.mic_none,
                    color: state.isVoiceListening ? Colors.redAccent : Colors.white,
                  ),
                ),
                IconButton(
                  tooltip: state.scenarioId == null ? 'Validation tests' : 'Finish test ${state.scenarioId}',
                  visualDensity: VisualDensity.compact,
                  onPressed: () => state.scenarioId == null
                      ? _showScenarios(context)
                      : context.read<AppState>().stopScenario(),
                  icon: Icon(state.scenarioId == null ? Icons.fact_check_outlined : Icons.stop_circle_outlined,
                      color: state.scenarioId == null ? Colors.white : Colors.amber),
                ),
                if (state.voiceLang != 'en' && state.voiceReady && !state.tamilVoiceInstalled)
                  const Text('Tamil voice not installed', style: TextStyle(color: Colors.amber, fontSize: 11)),
              ]),
              ]),
            ),
          ),
        ],
      ),
    );
  }

  Color _getStatusColor(String status) {
    if (status == 'CONNECTED') return Colors.greenAccent;
    if (status == 'REQUESTING PERMISSION') return Colors.orangeAccent;
    if (status == 'DISCONNECTED' || status == 'PERMISSION DENIED') return Colors.redAccent;
    return Colors.grey;
  }

  // What made the 1 m / 0.5 m "object ahead" check fire (so a screenshot shows the cause).
  static const _cocoNames = <int, String>{
    0: 'person', 13: 'bench', 24: 'backpack', 26: 'handbag', 28: 'suitcase', 39: 'bottle',
    41: 'cup', 56: 'chair', 57: 'sofa', 58: 'plant', 59: 'bed', 60: 'table', 61: 'toilet',
    62: 'TV', 63: 'laptop', 67: 'phone', 72: 'fridge', 73: 'book',
  };
  static String _aheadCause(String src) {
    if (src.isEmpty || src == 'none') return '';
    if (src.startsWith('det:')) {
      final id = int.tryParse(src.substring(4));
      return ' (${_cocoNames[id] ?? 'object $id'})';
    }
    return switch (src) {
      'obstacle' => ' (body height)',
      'head' => ' (head height)',
      'low' => ' (low object)',
      'trip' => ' (trip hazard)',
      _ => ' ($src)',
    };
  }

  // STOP (red) / CAUTION (amber) banner: Layer 1 safety or moving objects.
  Widget _motionAlertBanner(TelemetryData t) {
    final stop = t.alert == 'stop';
    final dist = t.alertDist > 0 ? ' ${t.alertDist.toStringAsFixed(1)} m' : '';
    final ttc = t.alertTtc > 0 ? ' · ${t.alertTtc.toStringAsFixed(1)} s' : '';
    final what = switch (t.alertReason) {
      'obstacle' => 'obstacle ahead$dist',
      'head' => 'head-height obstacle$dist',
      'dropoff' => 'step down$dist',
      'stairs' => 'stairs ahead$dist',
      'trip' => 'trip hazard$dist',
      'unknown' => 'unknown ground$dist',
      'nofloor' => 'cannot see the floor',
      'moving' => 'object approaching$ttc',
      'sensor' => 'camera problem',
      'ahead' => 'object directly ahead$dist${_aheadCause(t.aheadSrc)}',
      _ => 'hazard',
    };
    // Accessibility: readable when touched, but not a live region - the voice guide already
    // speaks alerts, TalkBack must not read them a second time.
    return Semantics(
      label: 'Alert: ${stop ? 'stop' : 'caution'}, $what',
      excludeSemantics: true,
      child: Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        color: stop ? Colors.redAccent : Colors.orangeAccent,
        borderRadius: BorderRadius.circular(8),
      ),
      child: Text(
        '${stop ? 'STOP' : 'CAUTION'} · $what',
        style: const TextStyle(color: Colors.black, fontSize: 18, fontWeight: FontWeight.bold),
      ),
      ),
    );
  }

  // Colours used by the native 3D view (view3d.cpp classColour).
  Widget _legend3d() {
    Widget item(Color c, String t) => Padding(
          padding: const EdgeInsets.symmetric(vertical: 1),
          child: Row(mainAxisSize: MainAxisSize.min, children: [
            Container(width: 12, height: 12, color: c),
            const SizedBox(width: 6),
            Text(t, style: const TextStyle(color: Colors.white, fontSize: 11)),
          ]),
        );
    return Container(
      padding: const EdgeInsets.all(6),
      decoration: BoxDecoration(color: Colors.black.withValues(alpha: 0.55), borderRadius: BorderRadius.circular(6)),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, mainAxisSize: MainAxisSize.min, children: [
        item(const Color(0xFF7887A5), 'wall'),
        item(const Color(0xFFB98C5F), 'object'),
        item(const Color(0xFFF0C828), 'door'),
        item(const Color(0xFFFF8C00), 'stairs / drop'),
        item(const Color(0xFFBE5AFF), 'person'),
        item(const Color(0xFFC83C32), 'obstacle'),
        item(const Color(0xFF28E63C), 'path'),
        item(const Color(0xFFFFAA28), 'path (wait)'),
        item(const Color(0xFF00E6FF), 'you'),
      ]),
    );
  }

  // Phase 7: pick a validation scenario (logged to a CSV on the phone).
  void _showScenarios(BuildContext context) {
    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.grey.shade900,
      builder: (ctx) => SafeArea(
        child: ListView(
          shrinkWrap: true,
          children: [
            const Padding(
              padding: EdgeInsets.all(12),
              child: Text('Validation test', style: TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold)),
            ),
            for (final (id, title, how) in AppState.scenarios)
              ListTile(
                leading: Text(id, style: const TextStyle(color: Colors.amber, fontWeight: FontWeight.bold)),
                title: Text(title, style: const TextStyle(color: Colors.white)),
                subtitle: Text(how, style: const TextStyle(color: Colors.white70)),
                onTap: () {
                  Navigator.pop(ctx);
                  context.read<AppState>().startScenario(id, title);
                },
              ),
          ],
        ),
      ),
    );
  }

  // Phase 5 guidance: one walking instruction with an arrow, path details below.
  Widget _planChip(TelemetryData t) {
    const icons = <String, IconData>{
      'straight': Icons.arrow_upward,
      'bear_left': Icons.north_west,
      'bear_right': Icons.north_east,
      'turn_left': Icons.turn_left,
      'turn_right': Icons.turn_right,
      'turn_left_soon': Icons.turn_left,
      'turn_right_soon': Icons.turn_right,
      'door': Icons.door_front_door,
      'stairs': Icons.stairs,
      'stop': Icons.pan_tool,
      'stop_left': Icons.turn_left,
      'stop_right': Icons.turn_right,
      'obstacle_ahead': Icons.pan_tool,
      'unverified': Icons.hourglass_top,
      'sensor_fault': Icons.videocam_off,
      'blocked': Icons.block,
    };
    final code = t.guideCode;
    final String text = t.guideText.isNotEmpty ? t.guideText : 'Mapping… walk slowly';
    final Color color = switch (code) {
      'stop' || 'stop_left' || 'stop_right' || 'obstacle_ahead' || 'sensor_fault' || 'blocked' => Colors.red.shade700,
      'stairs' || 'door' || 'unverified' => Colors.amber.shade800,
      'none' => Colors.grey.shade800,
      _ => Colors.green.shade700,
    };
    final detail = t.planOk ? 'path ${t.planLength.toStringAsFixed(1)} m' : '';
    return Semantics(
      label: 'Instruction: $text${detail.isNotEmpty ? ', $detail' : ''}',
      excludeSemantics: true,
      child: Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
      decoration: BoxDecoration(color: color.withValues(alpha: 0.9), borderRadius: BorderRadius.circular(18)),
      child: Row(mainAxisSize: MainAxisSize.min, children: [
        Icon(icons[code] ?? Icons.explore, color: Colors.white, size: 28),
        const SizedBox(width: 8),
        Flexible(child: Column(mainAxisSize: MainAxisSize.min, crossAxisAlignment: CrossAxisAlignment.start, children: [
          // Situation sentences are longer than the old two-word instructions: wrap.
          Text(text, maxLines: 3, overflow: TextOverflow.ellipsis,
              style: const TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold)),
          if (detail.isNotEmpty) Text(detail, style: const TextStyle(color: Colors.white70, fontSize: 12)),
        ])),
      ]),
      ),
    );
  }

  // Live D455 IMU readout: orientation + sample rates.
  Widget _imuPanel(ImuData imu) {
    const style = TextStyle(color: Colors.white, fontSize: 11, fontFeatures: [FontFeature.tabularFigures()]);
    // D455: show the measured sample rate, or NO DATA when no accel sample arrived in
    // the last 0.5 s (the IMU "runs" but delivers nothing; P/R would stay at 0).
    final d455 = imu.source == 'd455';
    final noData = d455 && (imu.ageMs < 0 || imu.ageMs > 500);
    if (imu.running && noData) {
      return Container(
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
        decoration: BoxDecoration(
          color: Colors.black.withValues(alpha: 0.6),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.orangeAccent),
        ),
        child: Text('IMU d455 · NO DATA', style: style.copyWith(color: Colors.orangeAccent)),
      );
    }
    final rate = d455 ? ' · ${imu.accelHz.toStringAsFixed(0)} Hz' : (imu.d455Failed ? ' (D455 failed)' : '');
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.6),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: imu.running ? Colors.cyanAccent.withValues(alpha: 0.6) : Colors.white24),
      ),
      child: imu.running
          // One line; tap to zero.
          ? Text('IMU${imu.source.isNotEmpty ? ' ${imu.source}' : ''}$rate  P ${imu.pitchDeg.toStringAsFixed(0)}°  R ${imu.rollDeg.toStringAsFixed(0)}°',
              style: style.copyWith(color: Colors.cyanAccent, fontSize: 11))
          : const Text('IMU off', style: style),
    );
  }



  static const _labelStyle = TextStyle(
    color: Colors.black,
    fontSize: 14,
    fontWeight: FontWeight.bold,
  );

  // Class name + distance tags for the native-drawn boxes. bbox is
  // [x, y, w, h] in 640x480 camera pixels; the feed is shown with
  // BoxFit.cover in a [view]-sized area, so map through the same scale/crop.
  // Labels are placed greedily in confidence order without overlapping:
  // above the box, else stepping down inside it, else just below it; a
  // label with no free spot is skipped. All stay below [topInset].
  List<Widget> _detectionLabels(BuildContext context,
      List<DetectionResult> detections, Size view, double topInset) {
    const srcW = 640.0, srcH = 480.0;
    const labelHeight = 24.0, labelMaxWidth = 220.0, padX = 6.0, gap = 2.0;
    final scale = (view.width / srcW) > (view.height / srcH)
        ? view.width / srcW
        : view.height / srcH;
    final dx = (view.width - srcW * scale) / 2;
    final dy = (view.height - srcH * scale) / 2;
    final maxY = view.height - labelHeight;

    final sorted = detections.where((d) => d.boundingBox.length >= 4).toList()
      ..sort((a, b) => b.confidence.compareTo(a.confidence));

    // Measure with the same theme font and text scale the Text widget uses.
    final measureStyle = DefaultTextStyle.of(context).style.merge(_labelStyle);
    final textScaler = MediaQuery.textScalerOf(context);

    final placed = <Rect>[];
    final widgets = <Widget>[];

    for (final det in sorted) {
      final left = det.boundingBox[0] * scale + dx;
      final top = det.boundingBox[1] * scale + dy;
      final bottom = (det.boundingBox[1] + det.boundingBox[3]) * scale + dy;

      // Box entirely cropped out of view: no label.
      if (bottom < topInset || top > view.height) continue;

      final distance = det.distanceMeters > 0
          ? '${det.distanceMeters.toStringAsFixed(1)} m'
          : '-- m';
      // Approaching objects show their closing speed.
      final motion = det.approaching ? ' · ↓${det.speed.toStringAsFixed(1)} m/s' : '';
      final text = '${det.objectName} · $distance$motion';

      // Fit the background to the text.
      final painter = TextPainter(
        text: TextSpan(text: text, style: measureStyle),
        maxLines: 1,
        textDirection: TextDirection.ltr,
        textScaler: textScaler,
      )..layout();
      // +2 px so rounding never triggers the ellipsis.
      final width = (painter.width.ceilToDouble() + 2 * padX + 2).clamp(0.0, labelMaxWidth);
      final x = left.clamp(0.0, (view.width - width).clamp(0.0, double.infinity));

      // Candidate rows: above the box, then down through the box, then below.
      final candidates = <double>[top - labelHeight];
      for (var y = top; y <= bottom - labelHeight; y += labelHeight + gap) {
        candidates.add(y);
      }
      candidates.add(bottom + gap);

      Rect? spot;
      for (final y in candidates) {
        if (y < topInset || y > maxY) continue;
        final r = Rect.fromLTWH(x, y, width, labelHeight);
        if (!placed.any((p) => p.inflate(gap).overlaps(r))) {
          spot = r;
          break;
        }
      }
      if (spot == null) continue;
      placed.add(spot);

      widgets.add(Positioned(
        left: spot.left,
        top: spot.top,
        child: Container(
          width: width,
          height: labelHeight,
          padding: const EdgeInsets.symmetric(horizontal: padX),
          alignment: Alignment.centerLeft,
          color: det.approaching
              ? (det.ttc > 0 && det.ttc < 2.0 ? Colors.redAccent : Colors.orangeAccent)
              : Colors.greenAccent,
          child: Text(
            text,
            maxLines: 1,
            overflow: TextOverflow.ellipsis,
            style: _labelStyle,
          ),
        ),
      ));
    }

    return widgets;
  }
}
