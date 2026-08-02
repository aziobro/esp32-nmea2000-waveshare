# ICM-20948 hardware test worksheet

Fill in one row per observation while running through
`doc/IcmPhysicalTestProcedure.md`. A CSV version of this same table
(`IcmHardwareTestWorksheet.csv`) is easier to import into a spreadsheet
for plotting deviation curves.

| Test # | Boat heading (reference) | DMP heading | Compass heading | Fusion heading | Output heading | Roll | Pitch | Mag magnitude | Engine | Alternator | Autopilot | VHF | Other loads | Quality | Rejection flags | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 6 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 7 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 8 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 9 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 10 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 11 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| 12 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |

Column notes:
- **Boat heading (reference)**: from a trusted steering compass, chart
  feature, or GPS COG at speed (not used as a calibration input per the
  project's own instruction, only as an independent cross-check here).
- **DMP/Compass/Fusion heading**: the three candidate rows on the IMU web
  tab, read simultaneously.
- **Output heading**: the tab's top "Heading (output), calibrated" row -
  whichever candidate `icmHeadingMode` actually selected.
- **Engine/Alternator/Autopilot/VHF/Other loads**: on/off (or a brief
  description of the load) at the moment of the reading - this is what
  steps 12-18 of the test procedure are specifically checking.
- **Quality/Rejection flags**: copy directly from the IMU tab's
  corresponding rows.
