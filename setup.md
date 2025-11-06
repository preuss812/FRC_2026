# New Computer Setup

Install [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)

Latest [Phoenix CTRE](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/)
- right click `build.gradle` in vscode and select > Manage Vendor Libraries > Install New Vendor Libraries (Offline) > Update CTRE to v5

Getting [PhotonVision](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html)
- Same process as Phoenix CTRE, but select Install New Vendor Libraries **(Online)** and paste this link `https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json`

Your code should now have a "BUILD SUCCESSFUL" when you try to build (command pallette > Build Robot Code)