```text
Module                  Status          Lead          Notes
------                  ------          ----          -----
dsPIC                   [Mainline]      Doug          Requires PCB work-arounds for programming connector and 3.3V regulator circuits
LEDs                    [Mainline]      Doug
Radio                   [Integrated]    Doug          Requires bug fixes in at86rf.c, commits pending
Motor control           [Verified]      Pullin, Doug  Requires PCB work-around for ground to one DRV8833
DataFlash               [Unverified]    Doug, fgb
BEMF sensing            [Unverified]    Pullin
IMU (MPU-6000)          [Unverified]    RJ 
OMAP Backpack           [Unverified]    Ryan          Requires PCB revision
Low battery IRQ         [Not started]   fgb
OVCam                   [Not started]   fgb
AMS hall effect         [Not started]   Duncan        Requires PCB revision
FeelerProc              [Not started]   Jaakko        Requires PCB revision
Radio TX/RX LEDs        [Not started]   Unclaimed
IMU IRQ                 [Not started]   Unclaimed
Battery level sensing   [Not started]   Unclaimed

[Mainline]      Modifications are integrated and have been merged into the
                imageproc-lib mainline.
[Integrated]    Modifications done and integrated in this branch. Mainline
                pending.
[Complete]      Code modifications are complete and pending integration.
[Verified]      Hardware functionality has been verified, but code
                modifications and testing are still in progress.
[Unverified]    Hardware functionality not yet verified
[Not started]   Nobody is working on this, yet.
```
