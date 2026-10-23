import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:hsv2_lite/main.dart';

void main() {
  testWidgets('App starts and shows START button', (WidgetTester tester) async {
    await tester.pumpWidget(const HSV2LiteApp());

    expect(find.text('NATIVE CAMERA FEED PLACEHOLDER'), findsOneWidget);
    expect(find.text('START'), findsOneWidget);
  });
}
