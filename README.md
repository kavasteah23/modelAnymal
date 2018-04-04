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

## Built With

* [ROSKinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu/) - Środowisko ROS wersja Kinetic


## Authors

* **Marian Wojtkowiak** 
* **Przemysław Muszyński** - [kavasteah23](https://github.com/kavasteah23)


