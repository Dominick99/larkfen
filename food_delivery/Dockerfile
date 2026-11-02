FROM python:3.11-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 MPLCONFIGDIR=/tmp/matplotlib
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . ./food_delivery
RUN useradd --create-home --uid 10001 simulator && mkdir /artifacts && chown simulator:simulator /artifacts
USER simulator
ENTRYPOINT ["python", "-m", "food_delivery"]
CMD ["--output", "/artifacts"]
