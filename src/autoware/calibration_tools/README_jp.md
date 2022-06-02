# calibration_tools

ディレクトリ構成はオーナーに相談して下さい。

ブランチ構成
main: ROS2
ros1: ROS1

ROS1 パッケージは徐々に ROS2 ポーティングを進めていくので、新規開発分は ROS2 でお願いします。
既に開発が済んだものだけ ROS1 ブランチに投入してください。

```txt
├── README.md
├── control
|   └── stop_accel_evaluator
└── vehicle
    ├── tier4_calibration_msgs
    ├── calibration_adapter
    ├── estimator_utils
    └── parameter_estimator
        └── README.md
```

## Evaluation

### result

- estimated result:
- estimated result stddev:
- estimated result stddev:

### error

- error
- error mean
- error stddev

_is calibration finished_ : bool
