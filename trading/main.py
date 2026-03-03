#!/usr/bin/env python3
"""
Algorithmic Trading System - CLI entry point.

Usage examples
--------------
# Run all strategies on synthetic data (default)
    python -m trading.main

# Load data from a CSV file
    python -m trading.main --csv prices.csv --symbol AAPL

# Tune a strategy
    python -m trading.main --strategy rsi --rsi-period 10 --oversold 25 --overbought 75

# Show trade log
    python -m trading.main --strategy ma --log
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime

from .backtest import Backtester
from .data import generate_synthetic_data, load_csv
from .risk import RiskParams
from .strategies import (
    BollingerBandStrategy,
    MACDStrategy,
    MACrossoverStrategy,
    RSIStrategy,
)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Algorithmic Trading System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Data source
    p.add_argument("--csv", metavar="FILE", help="Load OHLCV data from a CSV file.")
    p.add_argument("--symbol", default="DEMO", help="Ticker symbol name (default: DEMO).")
    p.add_argument(
        "--days", type=int, default=504, help="Synthetic data days (default: 504 ≈ 2 yrs)."
    )
    p.add_argument("--seed", type=int, default=42, help="Random seed for synthetic data.")

    # Strategy selection
    p.add_argument(
        "--strategy",
        choices=["ma", "rsi", "bb", "macd", "all"],
        default="all",
        help="Strategy to run (default: all).",
    )

    # MA Crossover params
    p.add_argument("--fast", type=int, default=20, help="Fast MA period (default: 20).")
    p.add_argument("--slow", type=int, default=50, help="Slow MA period (default: 50).")

    # RSI params
    p.add_argument("--rsi-period", type=int, default=14, help="RSI period (default: 14).")
    p.add_argument("--oversold", type=float, default=30.0, help="RSI oversold level.")
    p.add_argument("--overbought", type=float, default=70.0, help="RSI overbought level.")

    # Bollinger Band params
    p.add_argument("--bb-period", type=int, default=20, help="Bollinger Band period.")
    p.add_argument("--bb-std", type=float, default=2.0, help="Bollinger Band std multiplier.")

    # MACD params
    p.add_argument("--macd-fast", type=int, default=12)
    p.add_argument("--macd-slow", type=int, default=26)
    p.add_argument("--macd-signal", type=int, default=9)

    # Portfolio / risk
    p.add_argument(
        "--capital", type=float, default=100_000.0, help="Initial capital (default: 100,000)."
    )
    p.add_argument(
        "--commission", type=float, default=0.001, help="Commission rate (default: 0.1%%)."
    )
    p.add_argument(
        "--risk-pct",
        type=float,
        default=0.01,
        help="Risk per trade as fraction of equity (default: 0.01 = 1%%).",
    )
    p.add_argument(
        "--max-dd",
        type=float,
        default=0.20,
        help="Maximum drawdown before halting (default: 0.20 = 20%%).",
    )

    # Output
    p.add_argument("--log", action="store_true", help="Print the trade log after the report.")

    return p


def main(argv: list | None = None) -> None:
    args = build_parser().parse_args(argv)

    # ---- Load market data ----
    if args.csv:
        print(f"Loading data from {args.csv} ...")
        market_data = load_csv(args.symbol, args.csv)
    else:
        print(f"Generating {args.days} days of synthetic data for '{args.symbol}' ...")
        market_data = generate_synthetic_data(
            symbol=args.symbol,
            start=datetime(2020, 1, 1),
            days=args.days,
            seed=args.seed,
        )

    print(f"Loaded {len(market_data.bars)} bars  "
          f"({market_data.bars[0].timestamp.date()} → {market_data.bars[-1].timestamp.date()})\n")

    # ---- Build strategy list ----
    risk_params = RiskParams(
        risk_per_trade_pct=args.risk_pct,
        max_drawdown_pct=args.max_dd,
    )

    strategy_map = {
        "ma": MACrossoverStrategy(args.fast, args.slow),
        "rsi": RSIStrategy(args.rsi_period, args.oversold, args.overbought),
        "bb": BollingerBandStrategy(args.bb_period, args.bb_std),
        "macd": MACDStrategy(args.macd_fast, args.macd_slow, args.macd_signal),
    }

    if args.strategy == "all":
        strategies = list(strategy_map.values())
    else:
        strategies = [strategy_map[args.strategy]]

    # ---- Run backtests ----
    backtester = Backtester(
        initial_capital=args.capital,
        commission_rate=args.commission,
        risk_params=risk_params,
    )

    for strategy in strategies:
        report = backtester.run(strategy, market_data)
        print(report)
        if args.log and report.trade_log:
            print(f"\n  Trade Log ({len(report.trade_log)} entries):")
            for entry in report.trade_log:
                print(f"    {entry}")
        print()


if __name__ == "__main__":
    main()
