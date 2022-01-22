Voix off de la vidéo
====================

Introduction
------------

Bonjour à tous, dans cette vidéo, nous allons vous présenter notre projet réalisé dans le module de "Logiciels et architecture pour la robotique mobile" à l'IMT Nord Europe. Nous avons pu découvrir ROS en contrôlant un robot mobile. Le projet se compose de deux parties : la navigation, à savoir la cartographie de l'espace et le déplacement autonome du robot grâce à un capteur laser de type Lidar, et la vision robotique grâce à un capteur 3D embraquant une caméra couleur et une caméra de profondeur, qui servira à détecter et placer sur la carte des bouteilles noires.

Pendant la vidéo
----------------

**Pendant qu'on choisis la position sur Rviz** La cartographie est réalisée avec gmapping, un algorithme qui crée une carte de l'environnement à partir des données du laser, et qui localise le robot dessus. Le déplacement se fait ensuite avec move_base, un package à qui l'on peut donner une position, et qui va déterminer le meilleur chemin possible pour y arriver en évitant les obstacles, et qui se charge d'envoyer les commandes de vitesse adéquates au robot.

**Après que le robot ait commencé à avancer** Ici, nous avons demandé au robot de se rendre à l'autre bout de l'arène. Sur son chemin, la caméra va voir les bouteilles noires. La détection se fait grâce à la méthode de Haar : nous avons entrainé un modèle à reconnaitre ces bouteilles à l'aide d'une grande base de données d'images qui ont été prises directement avec le robot. Le capteur 3D est aussi équipé d'une caméra de profondeur. Combinées à la position des bouteilles sur l'image, ces informations nous permettent de déduire leurs coordonnées dans l'environnement. Chaque bouteille est ainsi ajoutée à une liste.

**Juste avant un des signaux sonores** A chaque fois qu'une bouteille est détectée, une LED s'allume et lorsqu'elle est validée, le robot émet un signal sonore **signal sonore** et un marqueur est placé sur la carte. Cette liste est d'ailleurs constamment envoyée à move_base pour permettre au robot d'éviter les bouteilles.

**Pendant le chemin du retour** Les bouteilles nouvellement détectées ne sont considérées comme valides qu'après avoir été détectées un nombre de fois suffisant. Cela permet de limiter les faux positifs. Une bouteille détectée mais déja présente dans la liste sera simplement mise à jour, ce qui permet à l'algorithme de mémoriser chaque bouteille de façon unique.

Capture d'écran Rviz
--------------------

Comme vous pouvez le voir, les trois bouteilles disposées dans l'arène ont été correctement détectées, et se retrouvent sur la carte sous forme de cubes verts.

Launch file
-----------

Le launch file lance les éléments suivants :
1. Les noeuds appropriés pour le contrôle du robot
2. Le noeud du capteur laser
3. Le noeud Gmapping, pour cartographier la zone
4. Le noeud move_base, pour le déplacement autonome
5. Les noeuds de la caméra
6. Le noeud de la détection des bouteilles, le script python qui les détecte et mémorise leur position.
7. Rviz, pour le retour visuel concernant la carte, l'état de move_base et la position des bouteilles.

rqt_graph
---------

Pas besoin de parler

Conclusion
----------

Merci
