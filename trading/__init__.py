"""
Algorithmic Trading System
==========================

A self-contained, zero-dependency algorithmic trading framework.

Modules
-------
data        : Market data (OHLCV bars), synthetic data generator, CSV loader.
indicators  : SMA, EMA, RSI, Bollinger Bands, MACD, ATR.
strategies  : MA Crossover, RSI, Bollinger Band, MACD strategies.
portfolio   : Position & cash tracking, order execution, equity curve.
risk        : Position sizing, stop-loss calculation, drawdown monitoring.
engine      : Trading engine that wires strategy → portfolio → risk.
backtest    : Backtester and PerformanceReport with key statistics.

Quick start
-----------
    from trading.data import generate_synthetic_data
    from trading.strategies import MACrossoverStrategy
    from trading.backtest import Backtester

    data = generate_synthetic_data("DEMO", start=datetime(2020, 1, 1), days=504)
    report = Backtester(initial_capital=100_000).run(MACrossoverStrategy(20, 50), data)
    print(report)
"""
