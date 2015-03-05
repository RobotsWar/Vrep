# V-REP

Tout d'abord:

```
apt-get install libzmq3-dev
```

Pour faire marcher la simulation:

* Cloner ce dépôt
* Télécharger vrep et le placer dans le sous-dossier Vrep de ce dépôt
* Aller dans Client/ puis taper "mkdir build; cd build; cmake ..; make"
* Aller dans Simulator/, puis taper "mkdir build; cd build; cmake ..; make"
* Dans Simulator/build, lancer ./vrep puis ./Client localhost 4242
* Clonner Soccer et dans AI taper "mkdir build; cd build; cmake .."
* Vous pouvez lancer AI dans Simulator/build


