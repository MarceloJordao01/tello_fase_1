# tello_fase_1

Esse é o node principal para rodar o navigator do tello
Ele simplesmente pega as configs em `params.yaml` e vai para cada waypoint esperando um certo tempo.

Caso queira mudar os waypoints, basta mudar no `params.yaml` 

para fazer o tello ir para o proximo waypoint mantenha como true o topico `move_next_waypoint`

```bash
ros2 topic pub /tello_navigator/move_next_waypoint std_msgs/Bool "{data: true}" --once
```


para ver se o tello ja chegou no waypoint

```bash
ros2 topic echo /tello_navigator/reach_waypoint
```
