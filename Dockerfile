FROM ghcr.io/dzmitry-kulik/stm32-ci-base:latest AS builder
WORKDIR /app
COPY . .

# Сборка прошивки
RUN --mount=type=cache,target=/root/.cache/ccache \
    cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    ninja -C build -j$(nproc)

# Копирование итогового артефакта
FROM scratch AS artifacts
COPY --from=builder /app/build/ADC_DMA.elf /