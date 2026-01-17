// package frc.robot.utils.GameManager;

// import frc.robot.Robot;
// import frc.robot.utils.Align.RelevantPosition;

// public enum Objectives {

//     CLIMB {
//         @Override
//         public Objective build(Robot robot) {
//             return new Objective(
//                 robot.CLIMB,
//                 ActionSequences.CLIMB.build(robot),
//                 () -> true,
//                 () -> false
//             );
//         }
//     },

//     SCORE_L4 {
//         @Override
//         public Objective build(Robot robot) {
//             return new Objective(
//                 null, // grid position if needed
//                 ActionSequences.SCORE_L4.build(robot),
//                 () -> true,
//                 () -> false
//             );
//         }
//     };

//     public abstract Objective build(Robot robot);
// }
