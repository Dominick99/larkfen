FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY simulation/requirements.txt ./simulation/requirements.txt
RUN pip install --no-cache-dir -r simulation/requirements.txt

COPY drone_protocol ./drone_protocol
COPY companion-computer/companion_computer ./companion-computer/companion_computer
COPY flight-controller/flight_controller ./flight-controller/flight_controller
COPY simulation/drone_sim ./simulation/drone_sim
COPY tests ./tests

ENV PYTHONPATH=/app:/app/simulation:/app/companion-computer:/app/flight-controller

RUN useradd --create-home --uid 10001 simulator \
    && mkdir -p /artifacts \
    && chown simulator:simulator /artifacts

USER simulator

ENTRYPOINT []
CMD ["python", "-m", "drone_sim", "--output", "/artifacts/flight.mp4"]
