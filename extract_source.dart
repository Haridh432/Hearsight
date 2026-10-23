import 'dart:io';
import 'package:archive/archive.dart';
import 'package:archive/archive_io.dart';

Future<void> main() async {
  final zipFile = File('android_realsense_build/headers.zip');
  if (!zipFile.existsSync()) {
    print('Source zip not found.');
    return;
  }
  
  print('Extracting librealsense source code...');
  final zipBytes = zipFile.readAsBytesSync();
  final zipArchive = ZipDecoder().decodeBytes(zipBytes);
  
  final outDir = Directory('android/app/src/main/cpp/librealsense');
  if (!outDir.existsSync()) outDir.createSync(recursive: true);
  
  for (final file in zipArchive) {
    if (file.isFile) {
      // The zip extracts to librealsense-2.50.0/...
      // We want to strip the first directory
      final firstSlash = file.name.indexOf('/');
      if (firstSlash != -1) {
        final relativePath = file.name.substring(firstSlash + 1);
        final outFile = File('${outDir.path}/$relativePath');
        if (!outFile.parent.existsSync()) outFile.parent.createSync(recursive: true);
        outFile.writeAsBytesSync(file.content as List<int>);
      }
    }
  }

  print('Source extraction complete!');
}
