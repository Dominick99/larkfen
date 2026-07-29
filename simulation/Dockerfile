FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY drone_sim ./drone_sim

RUN useradd --create-home --uid 10001 simulator \
    && mkdir -p /artifacts \
    && chown simulator:simulator /artifacts

USER simulator

ENTRYPOINT []
CMD ["python", "-m", "drone_sim", "--output", "/artifacts/flight.mp4"]
