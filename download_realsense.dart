import 'dart:io';
import 'package:archive/archive.dart';
import 'package:archive/archive_io.dart';

Future<void> main() async {
  final aarFile = File('android/app/libs/librealsense.aar');
  if (!aarFile.existsSync()) {
    print('AAR file not found at ${aarFile.path}');
    return;
  }
  
  print('Extracting local AAR...');
  final aarBytes = aarFile.readAsBytesSync();
  final aarArchive = ZipDecoder().decodeBytes(aarBytes);
  
  final libsDir = Directory('android/app/src/main/cpp/libs');
  if (!libsDir.existsSync()) libsDir.createSync(recursive: true);
  
  bool foundHeaders = false;

  for (final file in aarArchive) {
    if (file.isFile) {
      if (file.name.startsWith('jni/')) {
        // e.g. jni/arm64-v8a/librealsense2.so
        final parts = file.name.split('/');
        if (parts.length == 3 && parts[2].endsWith('.so')) {
          final arch = parts[1];
          final archDir = Directory('${libsDir.path}/$arch');
          if (!archDir.existsSync()) archDir.createSync(recursive: true);
          final outFile = File('${archDir.path}/${parts[2]}');
          outFile.writeAsBytesSync(file.content as List<int>);
          print('Extracted ${outFile.path}');
        }
      } else if (file.name.contains('include/librealsense2/')) {
        foundHeaders = true;
      }
    }
  }

  print('AAR Extraction Complete.');
  if (!foundHeaders) {
    print('NOTE: The AAR did not contain C++ headers. We must download them separately if they are not present.');
  }
}
