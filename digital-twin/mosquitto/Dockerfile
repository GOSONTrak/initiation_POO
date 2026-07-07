FROM eclipse-mosquitto:2.0

# Copier la configuration personnalisée
COPY mosquitto.conf /mosquitto/config/mosquitto.conf

# Créer les dossiers nécessaires
RUN mkdir -p /mosquitto/data /mosquitto/log && \
    chown -R mosquitto:mosquitto /mosquitto

# Exposer les ports MQTT
EXPOSE 1883
EXPOSE 9001

# Volume pour les données et logs
VOLUME ["/mosquitto/data", "/mosquitto/log"]

# Lancer Mosquitto
CMD ["/usr/sbin/mosquitto", "-c", "/mosquitto/config/mosquitto.conf"]
