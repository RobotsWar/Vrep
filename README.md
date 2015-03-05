# V-REP

Tout d'abord:

apt-get install libzmq3-dev

Pour faire marcher la simulation:

1) Cloner ce dépôt
2) Télécharger vrep et le placer dans le sous-dossier Vrep de ce dépôt
3) Aller dans Client/ puis taper "mkdir build; cd build; cmake ..; make"
4) Aller dans Simulator/, puis taper "mkdir build; cd build; cmake ..; make"
5) Dans Simulator/build, lancer ./vrep puis ./Client localhost 4242
6) Clonner Soccer et dans AI taper "mkdir build; cd build; cmake .."
7) Vous pouvez lancer AI dans Simulator/build


