import cv2

# Funktion zum Erkennen der Ampelfarben
def erkennen_ampelfarben(frame):
    # Bildgröße anpassen, falls erforderlich
    frame = cv2.resize(frame, (640, 480))
  
    # Farbbereiche für die Ampelfarben definieren
    lower_red = (0, 0, 100)
    upper_red = (80, 80, 255)
  
    lower_yellow = (0, 100, 100)
    upper_yellow = (80, 255, 255)
  
    lower_green = (0, 100, 0)
    upper_green = (80, 255, 80)
  
    # Bild in den HSV-Farbraum konvertieren
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
  
    # Farben in den definierten Bereichen filtern
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
  
    # Farberkennung für jede Ampelfarbe
    if cv2.countNonZero(mask_red) > 1000:
        return "Rot"
    elif cv2.countNonZero(mask_yellow) > 1000:
        return "Gelb"
    elif cv2.countNonZero(mask_green) > 1000:
        return "Grün"
    else:
        return "Keine Ampelfarbe erkannt"

# Video-Stream öffnen
cap = cv2.VideoCapture(0)

while True:
    # Einzelbild aus dem Video-Stream lesen
    ret, frame = cap.read()
  
    # Ampelfarben erkennen
    ampelfarbe = erkennen_ampelfarben(frame)
  
    # Ampelfarbe ausgeben
    print("Erkannte Ampelfarbe: " + ampelfarbe)
  
    # Bild anzeigen
    cv2.imshow("Ampelerkennung", frame)
  
    # Programm beenden, wenn 'q' gedrückt wird
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Video-Stream und Fenster schließen
cap.release()
cv2.destroyAllWindows()