# modelAnymal

Model robota ANYmal (ETH) w symulatorze PUT.

## Wprowadzenie

Projekt mający na celu przeniesienie modelu robota kroczącego ANYmal środowisku ROS z nakładką ETH (Swiss Federal Institute of Technology in Zurich) do symulatora PUT (Poznan University of Technology), a następnie zaimplementowanie modułu esytmacji poślizgu.

### Wymagania
* Środowisko ROS kinetic 
* Symulator PUT wraz z niezbędnymi pakietami 
* Symulator ETH wraz z niezbędnymi pakietami

### Instalacja komponentów

Instalacja symulatorów odbywa się według dołączonych dokumentacji.


## Uruchomienie symulatorów

* Symulator ETH
Uruchomienie podstawowej symulacji ANYmal:
```
  roslaunch anymal_sim sim.launch
```
* Symulator PUT
Uruchomienie przykładu dostępne w katalogu:
```
  Samples/Bin/SimpleViewer
```

## Niezbędne pakiety

* [ROSKinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu/) - Środowisko ROS wersja Kinetic

## Naprawa błędów

* Brak lub błędy w plikach nagłówkowych dla symulatora ETH (pakiet free_gait)
```
  free_gait/free_gait_core/include/free_gait_core/executor/Executor.hpp
  free_gait/free_gait_core/include/free_gait_core/executor/executor.hpp
  free_gait/free_gait_core/include/free_gait_core/step/step.hpp
  free_gait/free_gait_core/include/free_gait_core/step/Step.hpp
```
* Brak biblioteki Octomap

Dostępne na https://github.com/leggedrobotics/free_gait



## Autorzy

* **Marian Wojtkowiak** 
* **Przemysław Muszyński** - [kavasteah23](https://github.com/kavasteah23)


