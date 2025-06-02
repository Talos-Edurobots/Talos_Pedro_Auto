//package pedroPathing.simulation;
//
//import pedroPathing.examples.TeleOpMain;
//
//public class Simulation {
//    public static void main(String[] args) {
//        TestableTeleOpMain teleOp = new TestableTeleOpMain();
//        // Simulate driver starting the OpMode
//        new Thread(() -> {
//            try {
//                teleOp.runOpMode();
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//        }).start();
//
//        // Simulate inputs
//        try {
//            Thread.sleep(2000); // wait for init
//            teleOp.setActive(true);
//
//            // Simulate gamepad2 pressing up stick (expanding viper)
//            teleOp.gamepad2.left_stick_y = -1;
//            Thread.sleep(1000);
//            teleOp.gamepad2.left_stick_y = 0;
//
//            Thread.sleep(2000);
//           teleOp.setActive(false); // stop OpMode
//
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//    }
//}
