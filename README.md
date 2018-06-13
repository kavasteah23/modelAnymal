# modelAnymal

Model robota ANYmal (ETH) w symulatorze PUT.

## Wprowadzenie

Projekt mający na celu przeniesienie modelu robota kroczącego ANYmal środowisku ROS z nakładką ETH (Swiss Federal Institute of Technology in Zurich) do symulatora PUT (Poznan University of Technology), a następnie zaimplementowanie modułu esytmacji poślizgu.

### Wymagania
* Środowisko ROS kinetic 
* Symulator PUT wraz z niezbędnymi pakietami 
* Symulator ETH wraz z niezbędnymi pakietami

### Instalacja komponentów

Instalacja symulatorów odbywa się według dołączonych instrukcji i dokumentacji.

## Uruchomienie symulatorów

* Symulator ETH (wersja ROS)
Uruchomienie podstawowej symulacji ANYmal:
```
  source ~/catkin_ws/devel/setup.bash
  roslaunch anymal_sim sim.launch
```
* Symulator PUT
Uruchomienie programu:
```
  ./build/bin/demoVisualizer
```
## Pakiet ROS do badania poślizgu robota ANYmal
* Instalacja pakietu dla przestrzeni roboczej robota ANYmal
```
catkin build slip_measure
```
* Uruchomienie pakietu (po uruchomieniu symulatora ETH)
```
rosrun slip_measure slip_measure_node
```
* Odczyt wartości poprzez nasłuchiwanie topiców dla każdej nogi (slip_leg0,slip_leg1,slip_leg2,slip_leg3)
```
rostopic echo slip_leg0
```
## Niezbędne pakiety

* [ROSKinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu/) - Środowisko ROS wersja Kinetic
* [OpenCV] - Najnowsza wersja (3.4.x)
* [RealSense] - librealsense (1.12.x)

## Naprawa błędów

* Brak lub błędy w plikach nagłówkowych dla symulatora ETH (pakiet free_gait), poprawne dostępne w repozytorium https://github.com/leggedrobotics/free_gait
```
  free_gait/free_gait_core/include/free_gait_core/executor/Executor.hpp
  free_gait/free_gait_core/include/free_gait_core/executor/executor.hpp
  free_gait/free_gait_core/include/free_gait_core/step/step.hpp
  free_gait/free_gait_core/include/free_gait_core/step/Step.hpp
```


* Brak biblioteki Octomap





## Autorzy

* **Marian Wojtkowiak** 
* **Przemysław Muszyński** - [kavasteah23](https://github.com/kavasteah23)


