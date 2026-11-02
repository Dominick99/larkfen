FROM python:3.11-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 MUJOCO_GL=osmesa
WORKDIR /app
RUN apt-get update && apt-get install --no-install-recommends --yes libgl1 libglfw3 libosmesa6 && rm -rf /var/lib/apt/lists/*
COPY car_following/requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt
COPY robotics/src /app/robotics/src
COPY mujoco_sim/src /app/mujoco_sim/src
COPY car_following /app/car_following
ENV PYTHONPATH=/app/robotics/src:/app/mujoco_sim/src:/app
RUN useradd --create-home --uid 10001 simulator && mkdir /artifacts && chown simulator:simulator /artifacts
USER simulator
ENTRYPOINT ["python", "-m", "car_following"]
CMD ["--output", "/artifacts"]
