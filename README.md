# larm_moutarde

Branche pour le challenge 1 : déplacer le robot dans un espace clos sans entrer en collision

 /!\ Le projet nécessite le paquet mb6-tbot

Le paquet contient 2 fichiers launch, un pour la simulation et un pour une exécution sur un turtlrbot.

Dans les deux cas un rviz est lancé pour permettre de voir ce que le robot détecte grâce à son laser.

Notre stratégie de mouvement consiste à faire accélerer le robot de manière constante en ligne droite lorsqu'il n'y a plus d'obstacle devant lui. Ceci afin de gagner en vitesse dans les espaces vides sans produire de mouvements brusques. De même, il décellère graduellement lorsqu'il se raproche d'un obstacle. 
Ensuite, pour ce qui est de la direction, Lorsqu'il est proche d'un obstacle, il tourne pour l'éviter. Dans un premier temps, en continuant d'avancer puis en mode stationnaire si l'obstacle est trop proche.
La combinaison entre la distance d'arrêt à l'avant et la distance de rotation lui permet de s'éloigner de l'obstacle et de ne pas le longer. Cela permet d'empêcher le robot de simplement suivre les murs une fois qu'il en a rencontré un dans le cas où les angles internes à l'espace clos sont tous inférieurs à 180°.


