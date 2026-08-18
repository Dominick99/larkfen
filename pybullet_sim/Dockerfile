FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY src ./src

ENV PYTHONPATH=/app/src

RUN useradd --create-home --uid 10001 simulator \
    && mkdir -p /artifacts \
    && chown simulator:simulator /artifacts

USER simulator

ENTRYPOINT ["python", "-m", "pybullet_sim"]
CMD ["--output", "/artifacts/simulation.mp4"]
