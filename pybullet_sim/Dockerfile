FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY pybullet_sim/requirements.txt ./pybullet_sim/requirements.txt
RUN pip install --no-cache-dir -r pybullet_sim/requirements.txt

COPY robotics/src ./robotics/src
COPY robotics/tests ./robotics/tests
COPY pybullet_sim/src ./pybullet_sim/src
COPY pybullet_sim/tests ./pybullet_sim/tests

ENV PYTHONPATH=/app/robotics/src:/app/pybullet_sim/src

RUN useradd --create-home --uid 10001 simulator \
    && mkdir -p /artifacts \
    && chown simulator:simulator /artifacts

USER simulator

ENTRYPOINT ["python", "-m", "pybullet_sim"]
CMD ["--output", "/artifacts/simulation.mp4"]
