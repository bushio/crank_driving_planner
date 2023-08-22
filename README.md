# crank_driving_planner
A planner for Autoware that runs on an S-shaped crank.

# 実行手順
リポジトリをクローン
```
cd aichallenge_ws/src/
git clone git@github.com:bushio/crank_driving_planner.git
```
motion_planning.launch.py を置き換え
```
cp aichallenge_ws/src/crank_driving_planner/launch/motion_planning.launch.py aichallenge_ws/src/aichallenge_submit/autoware_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/
```
実行
```
ros2 launch crank_driving_planner crank_driving_planner.launch.xml
```