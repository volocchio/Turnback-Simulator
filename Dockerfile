FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

WORKDIR /app

# Runtime libraries for xhtml2pdf / reportlab. The pinned requirements
# publish manylinux wheels, so the image does not carry a compiler.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        libcairo2 \
        libpango-1.0-0 \
        libpangocairo-1.0-0 \
        libgdk-pixbuf-2.0-0 \
        shared-mime-info \
        fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY engine ./engine
COPY analysis ./analysis
COPY data/*.parquet ./data/
COPY turnback_app.py ./

EXPOSE 8501

CMD ["streamlit", "run", "turnback_app.py", \
     "--server.address=0.0.0.0", \
     "--server.port=8501", \
     "--server.headless=true", \
     "--logger.level=info", \
     "--browser.gatherUsageStats=false"]
