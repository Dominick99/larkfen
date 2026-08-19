FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MUJOCO_GL=osmesa

WORKDIR /app

RUN apt-get update \
    && apt-get install --no-install-recommends --yes libgl1 libglfw3 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

COPY mujoco_sim/requirements.txt ./mujoco_sim/requirements.txt
RUN pip install --no-cache-dir -r mujoco_sim/requirements.txt

COPY robotics/src ./robotics/src
COPY robotics/tests ./robotics/tests
COPY mujoco_sim/src ./mujoco_sim/src
COPY mujoco_sim/tests ./mujoco_sim/tests

ENV PYTHONPATH=/app/robotics/src:/app/mujoco_sim/src

RUN useradd --create-home --uid 10001 simulator \
    && mkdir -p /artifacts \
    && chown simulator:simulator /artifacts

USER simulator

ENTRYPOINT ["python", "-m", "mujoco_sim"]
CMD ["--output", "/artifacts/simulation.mp4"]
