# Turtlebot Potential Field
Applicazione dell'algoritmo del Potential Field al Turtlebot.

## DEMO
Sono presenti due demo, una animata e una no, di un simulatore che consente di vedere il percorso del robot.
Il simulatore lavora in metri.

* L'area di simulazione ipotizza le stesse dimensioni del campo in cui sarà utilizzato il robot.
* Il robot parte dalla posizione x,y = (0.105,0.105).
* Il target di destinazione del robot è x,y = (0.8,2.5)
* Il percorso contiene quattro ostacoli.

### Simulatore
+ Simulatore.py

![alt text](https://github.com/franzqat/TurtlebotPotentialField/blob/master/Figure_1.png "Demo non animata")

### Simulatore animato
+ Simulatore_animated.py
Versione animata del precedente simulatore

### Applicazione pratica
Sul turtlebot occorre caricare i files __test.py__ e __turtlebb.py__. e __potentialfield_control.py__

test.py è l'applicazione da eseguire sul turtlebot per mettere in pratica l'algoritmo.
* L'applicazione lavora in millimetri.
* Per eseguirla, una volta trasferito lo script sul robot, occorre lanciarlo tramite interprete python2
* Una volta avviato basta scrivere "go 800 2500"



## Librerie
Robot.py è la libreria usata per la simulazione del robot nel simulatore.

turtlebb.py è il driver del robot usato nell'applicazione

potentialfield_control.py è il controllore che implementa l'algoritmo di potential field.