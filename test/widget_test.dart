import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:hsv2_lite/main.dart';

void main() {
  testWidgets('App starts and shows START button', (WidgetTester tester) async {
    await tester.pumpWidget(const HSV2LiteApp());

    expect(find.text('WAITING FOR NATIVE REALSENSE FEED'), findsOneWidget);
    expect(find.text('START'), findsOneWidget);
  });
}
