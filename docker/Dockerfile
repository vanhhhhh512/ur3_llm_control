# Bai thuc hanh 02 - dieu khien UR3e bang LLM.
# Dung lai nen ROS 2 Humble + MoveIt 2 + Ignition Fortress da dung o bai 01,
# chi bo sung SDK goi API tuong thich OpenAI de noi toi 9Router.
FROM ur3e-draw:humble

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip \
 && python3 -m pip install --no-cache-dir "openai>=1.0,<2.0" \
 && rm -rf /var/lib/apt/lists/*

ENV NINEROUTER_BASE_URL=http://127.0.0.1:20128/v1 \
    IGN_PARTITION=ur3llm

WORKDIR /ws
