# aip_description.iv

aip_description for pilot.auto

## Settings

- Add each transformation between a base frame and a sensor into calibration yaml as below.

```yaml
# sample_sensor_calibration.yaml
# parent -> child
parent_name
  child_name
    x:
    y:
    z:
    roll:
    pitch:
    yaw:
```
