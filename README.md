# crank_driving_planner
A planner for Autoware that runs on an S-shaped crank.

# 実行手順
1. リポジトリをクローン
```
cd aichallenge_ws/src/aichallenge_submit/
git clone git@github.com:bushio/crank_driving_planner.git
```
2. motion_planning.launch.py の置き換え
```
cp aichallenge_ws/src/crank_driving_planner/launch/motion_planning.launch.py aichallenge_ws/src/aichallenge_submit/autoware_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/
```
3. aichallenge_submit.launch.xml の置き換え
```
cp aichallenge_ws/src/crank_driving_planner/launch/aichallenge_submit.launch.xml aichallenge_ws/src/aichallenge_submit/aichallenge_submit_launch/launch/
```

# 単体実行
- 下記を実行する。
```
ros2 launch crank_driving_planner crank_driving_planner.launch.xml
```