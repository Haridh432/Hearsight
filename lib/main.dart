import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'providers/app_state.dart';
import 'screens/home_screen.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const HSV2LiteApp());
}

class HSV2LiteApp extends StatelessWidget {
  const HSV2LiteApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => AppState()),
      ],
      child: MaterialApp(
        title: 'HSV2 Lite',
        theme: ThemeData(
          brightness: Brightness.dark,
          primarySwatch: Colors.teal,
          scaffoldBackgroundColor: const Color(0xFF121212),
        ),
        home: const HomeScreen(),
      ),
    );
  }
}
