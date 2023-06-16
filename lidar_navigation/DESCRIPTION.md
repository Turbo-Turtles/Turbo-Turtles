# LiDAR Navigation

Robert Schneider, 14.06.2023

## Grundfunktionen

Zum einfacheren Testen wurde eine Gazebo Simulation mit der AutoRace Track erstellt.

Und zum allgemeinen Nutzen der Daten wird der Navigation2 stack mit der slam_toolbox verwendet um eine Karte von der Welt zu erstellen und sich darin zu orientieren. 

## Tunnel Mission

Der Abschnitt funktioniert soweit. (Leider nicht in echt, da die Karte dort anders (versetzt) erstellt wird, als in der Simulation... Aber daran wird gearbeitet.)

Am Startpunkt wird die Position des Bots gespeichert und aus diesen Koordinaten wird dann später der Waypoint am Tunnelausgang berechnet (0.5 Meter hinterm Bot).

Wenn der Bot vor dem Tunneleingang steht soll die Schilderkennung später dann des entsprechende Schild erkennen und eine Message mit dem Namen der Mission (in dem Fall logischerweise "tunnel") senden. Diese Message wird von der Node für die Tunnel Mission empfangen und die Routine beginnt.

Dazu wird die InitialPose vom Roboter auf die aktuelle Position auf der Karte gesetzt. Dann wird ein erster Waypoint vor dem Bot generiert (auch ca. 0.5 Meter) und der am Anfang generierte Waypoint wird als zweites geladen. Diese Waypoints werden dann an den Navigation2 stack übergeben, welcher einen Pfad daran abfährt durch den Tunnel.

## Construction Mission

Hieran wird gerade gearbeitet. (Setzt auch voraus, dass die Karte vernünftig generiert wird. Siehe Tunnel Mission)

Idee ist, dass die Karte, welche mit SLAM erstellt wurde analysiert wird, um die groben Koordinaten der (drei) Hindernisse herauszufinden.

Dazu soll der two-pass Algorithmus zum Einsatzkommen, da er leicht zu implementieren ist und die Analyse nur einmal stattfinden muss, und somit Performance keine übergeordnete Rolle spielt.

Auf Basis dieser Koordinaten werden dann wieder Waypoints erstellt und an den Nav2 stack übergeben.

## Parking Mission

Steht noch aus.

Im Prinzip kann man hier vorgehen wie bei der Construction Mission und die Karte analysieren, um den besetzten Parkplatz zu ermitteln, man könnte aber auch die Rohdaten des LiDARs nutzen und nach dem Minimalwert suchen und in dieser Richtung währe dann der besetzte Parkplatz.