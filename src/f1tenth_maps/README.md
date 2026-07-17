# f1tenth_maps

slam_toolbox로 생성한 맵(pgm/yaml)을 여기 `maps/` 폴더에 넣습니다.

```
f1tenth_maps/
  maps/
    lab_track.pgm
    lab_track.yaml
```

`map_name` launch 인자와 파일명이 일치해야 합니다 (예: `map_name:=lab_track`).

한 번 저장한 맵은 이후 함부로 다시 slam_toolbox 매핑 모드로 덮어쓰지 말고,
localization 모드로만 불러와서 map->odom 좌표계를 고정하세요.
(중간발표 때 겪은 TF 불일치 문제가 여기서 재발하기 가장 쉬운 지점입니다.)
