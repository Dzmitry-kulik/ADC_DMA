FROM ghcr.io/dzmitry-kulik/stm32-ci-base:latest AS base
WORKDIR /app
COPY . .

# =========================================================================
# 1. Host Unit-тесты (Выполняются параллельно)
# =========================================================================
FROM base AS host_tests
RUN --mount=type=cache,target=/root/.cache/ccache \
    cmake -B build_host -G Ninja -DENABLE_COVERAGE=ON && \
    ninja -C build_host -j$(nproc) && \
    ./build_host/fsm_unit_tests && \
    gcovr --gcov-executable gcov-12 -r . --filter "Core/Src/protocol/src/FSM_parser.cpp" --fail-under-line 80

# =========================================================================
# 2. Тестовая прошивка + Renode (Выполняются параллельно)
# =========================================================================
FROM base AS renode_tests
RUN --mount=type=cache,target=/root/.cache/ccache \
    cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-DCI_RENODE_TEST" -DCMAKE_C_FLAGS="-DCI_RENODE_TEST" && \
    ninja -C build -j$(nproc) && \
    python3 tests/check_size.py

RUN renode --disable-xwt tests/test_board.resc & PID=$! && \
    sleep 3 && \
    python3 tests/tests_renode.py --url socket://localhost:4321 --timeout 0.5 && \
    python3 tests/benchmark.py --url socket://localhost:4321 --output build/benchmark_results.png && \
    python3 tests/stress_test.py --url socket://localhost:4321 --duration 10; \
    TEST_RESULT=$? ; \
    gdb-multiarch build/UART_DRIVER.elf -batch -x tests/ci_debug.gdb ; \
    kill $PID ; \
    exit $TEST_RESULT

# =========================================================================
# 3. Сборка БОЕВОЙ прошивки для железа (Выполняется параллельно)
# =========================================================================
FROM base AS prod_build
RUN --mount=type=cache,target=/root/.cache/ccache \
    cmake -B build_prod -G Ninja -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    ninja -C build_prod -j$(nproc)

# =========================================================================
# Финальный этап: собираем все результаты
# =========================================================================
FROM scratch AS artifacts
# Копируем прошивку из задачи prod_build
COPY --from=prod_build /app/build_prod/UART_DRIVER.elf /
# Копируем график из задачи renode_tests
COPY --from=renode_tests /app/build/benchmark_results.png /