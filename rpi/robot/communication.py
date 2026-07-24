import serial
import time


PORT = "/dev/cu.usbserial-1130"
BAUDRATE = 115200


class Robot:

    def __init__(self):
        self.serial = serial.Serial(
            PORT,
            BAUDRATE,
            timeout=0
        )

        self._buffer_reception = bytearray()

        # L'Arduino redémarre à l'ouverture du port et initialise les servos.
        # On attend son message PRET au lieu d'utiliser un délai fixe trop court.
        print("Connexion Arduino : attente de PRET...")
        debut = time.time()
        arduino_pret = False

        while time.time() - debut < 15:
            ligne = self.lire()

            if ligne != "":
                print("Arduino >", ligne)

                if ligne == "PRET":
                    arduino_pret = True
                    break

            time.sleep(0.01)

        if not arduino_pret:
            print("Attention : l'Arduino n'a pas envoyé PRET")

    def envoyer(self, angle, distance):
        angle = max(0, min(180, int(round(angle))))
        distance = max(0, int(round(distance)))

        message = f"{angle};{distance}\n"

        print("Envoi série >", repr(message))

        self.serial.reset_input_buffer()
        self._buffer_reception.clear()
        self.serial.write(message.encode("utf-8"))
        self.serial.flush()

    def envoyer_rotation(self, angle):
        """Tourne uniquement la base pour permettre une mesure visuelle."""
        angle = max(0, min(180, int(round(angle))))
        message = f"R;{angle}\n"

        print("Envoi rotation série >", repr(message))

        self.serial.reset_input_buffer()
        self._buffer_reception.clear()
        self.serial.write(message.encode("utf-8"))
        self.serial.flush()

    def envoyer_cycle(
        self,
        angle_prise,
        distance_prise,
        angle_depot,
        distance_depot,
    ):
        angle_prise = max(0, min(180, int(round(angle_prise))))
        distance_prise = max(0, int(round(distance_prise)))
        angle_depot = max(0, min(180, int(round(angle_depot))))
        distance_depot = max(0, int(round(distance_depot)))

        message = (
            f"{angle_prise};{distance_prise};"
            f"{angle_depot};{distance_depot}\n"
        )

        print("Envoi cycle série >", repr(message))

        self.serial.reset_input_buffer()
        self._buffer_reception.clear()
        self.serial.write(message.encode("utf-8"))
        self.serial.flush()

    def envoyer_remplacement(
        self,
        angle_vide,
        distance_vide,
        angle_depot,
        distance_depot,
        angle_plein,
        distance_plein,
        angle_destination,
        distance_destination,
    ):
        valeurs = [
            angle_vide,
            distance_vide,
            angle_depot,
            distance_depot,
            angle_plein,
            distance_plein,
            angle_destination,
            distance_destination,
        ]

        for index in (0, 2, 4, 6):
            valeurs[index] = max(0, min(180, int(round(valeurs[index]))))

        for index in (1, 3, 5, 7):
            valeurs[index] = max(0, int(round(valeurs[index])))

        message = ";".join(str(valeur) for valeur in valeurs) + "\n"

        print("Envoi remplacement série >", repr(message))

        self.serial.reset_input_buffer()
        self._buffer_reception.clear()
        self.serial.write(message.encode("utf-8"))
        self.serial.flush()

    def _envoyer_phase_remplacement(
        self,
        prefixe,
        angle_prise,
        distance_prise,
        angle_depot,
        distance_depot,
    ):
        """Envoie une moitié du remplacement au programme Arduino."""
        angle_prise = max(0, min(180, int(round(angle_prise))))
        distance_prise = max(0, int(round(distance_prise)))
        angle_depot = max(0, min(180, int(round(angle_depot))))
        distance_depot = max(0, int(round(distance_depot)))

        message = (
            f"{prefixe};{angle_prise};{distance_prise};"
            f"{angle_depot};{distance_depot}\n"
        )

        print("Envoi phase remplacement >", repr(message))

        self.serial.reset_input_buffer()
        self._buffer_reception.clear()
        self.serial.write(message.encode("utf-8"))
        self.serial.flush()

    def envoyer_vide_vers_depot(
        self,
        angle_vide,
        distance_vide,
        angle_depot,
        distance_depot,
    ):
        self._envoyer_phase_remplacement(
            "V",
            angle_vide,
            distance_vide,
            angle_depot,
            distance_depot,
        )

    def envoyer_plein_vers_destination(
        self,
        angle_plein,
        distance_plein,
        angle_destination,
        distance_destination,
    ):
        self._envoyer_phase_remplacement(
            "P",
            angle_plein,
            distance_plein,
            angle_destination,
            distance_destination,
        )

    def lire(self):
        # Lecture strictement non bloquante : on récupère seulement les octets
        # déjà reçus, puis on renvoie une ligne lorsqu'elle est complète.
        nombre_octets = self.serial.in_waiting

        if nombre_octets > 0:
            self._buffer_reception.extend(
                self.serial.read(nombre_octets)
            )

        fin_ligne = self._buffer_reception.find(b"\n")

        if fin_ligne == -1:
            return ""

        ligne = self._buffer_reception[:fin_ligne]
        del self._buffer_reception[:fin_ligne + 1]

        return ligne.decode("utf-8", errors="replace").strip()

    def attendre_fin(self, timeout=10):
        debut = time.time()

        while time.time() - debut < timeout:
            ligne = self.lire()

            if ligne != "":
                print("Arduino >", ligne)

            if ligne == "TERMINE":
                return True

            time.sleep(0.01)

        print("Erreur : aucune réponse TERMINE de l'Arduino")
        return False

    def fermer(self):

        self.serial.close()
