"""
Trading engine: converts signals into orders, manages stops.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

from .data import MarketData
from .portfolio import Portfolio
from .risk import RiskManager
from .strategies import BaseStrategy, Direction, Signal
from . import indicators


@dataclass
class ActiveStop:
    """Tracks a stop-loss for an open position."""
    symbol: str
    stop_price: float
    direction: str   # "BUY" = we are long; "SELL" = we are short


class TradingEngine:
    """
    Drives a single-symbol backtest / paper-trading session.

    Flow per bar:
      1. Update equity + risk manager.
      2. Check existing stops for exit.
      3. Evaluate new signals for entry.
      4. Record equity snapshot.
    """

    def __init__(
        self,
        strategy: BaseStrategy,
        portfolio: Portfolio,
        risk_manager: RiskManager,
    ) -> None:
        self.strategy = strategy
        self.portfolio = portfolio
        self.risk_manager = risk_manager
        self._stops: Dict[str, ActiveStop] = {}
        self.log: List[str] = []

    # ------------------------------------------------------------------
    # Main execution loop
    # ------------------------------------------------------------------

    def run(self, market_data: MarketData) -> None:
        """
        Execute the full strategy over the provided market data.
        """
        bars = market_data.bars
        closes = market_data.closes()
        highs = market_data.highs()
        lows = market_data.lows()

        # Pre-compute ATR for position sizing
        atr_vals = indicators.atr(highs, lows, closes, period=14)

        # Generate all signals upfront
        signals: Dict[int, List[Signal]] = {}
        for sig in self.strategy.generate_signals(market_data):
            signals.setdefault(sig.index, []).append(sig)

        for i, bar in enumerate(bars):
            current_price = bar.close
            ts = bar.timestamp
            prices = {market_data.symbol: current_price}

            # 1. Update risk manager
            equity = self.portfolio.equity(prices)
            self.risk_manager.update_equity(equity)

            # 2. Check stop-loss exits
            self._check_stops(market_data.symbol, bar, ts)

            # 3. Process signals for this bar
            bar_signals = signals.get(i, [])
            for sig in bar_signals:
                self._process_signal(
                    sig=sig,
                    symbol=market_data.symbol,
                    current_price=current_price,
                    atr=atr_vals[i],
                    equity=equity,
                    ts=ts,
                )

            # 4. Record equity
            self.portfolio.record_equity(ts, prices)

        # Close any remaining positions at the last price
        last_bar = bars[-1]
        self.portfolio.close_all(
            {market_data.symbol: last_bar.close}, last_bar.timestamp
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _process_signal(
        self,
        sig: Signal,
        symbol: str,
        current_price: float,
        atr: Optional[float],
        equity: float,
        ts: datetime,
    ) -> None:
        open_pos = len(self.portfolio.positions)

        if sig.direction == Direction.BUY:
            if not self.risk_manager.allow_trade(open_pos):
                self._log(ts, f"BLOCKED BUY {symbol} @ {current_price:.4f} (risk limit)")
                return
            if symbol in self.portfolio.positions:
                return  # Already long

            qty = self.risk_manager.position_size(current_price, equity, atr)
            qty = max(1.0, round(qty))

            filled = self.portfolio.buy(symbol, qty, current_price, ts)
            if filled:
                stop = self.risk_manager.stop_price(current_price, "BUY", atr)
                self._stops[symbol] = ActiveStop(symbol, stop, "BUY")
                self._log(ts, f"BUY  {qty:.0f} {symbol} @ {current_price:.4f}  stop={stop:.4f}")

        elif sig.direction == Direction.SELL:
            if symbol not in self.portfolio.positions:
                return  # Nothing to sell

            pos = self.portfolio.positions[symbol]
            filled = self.portfolio.sell(symbol, pos.quantity, current_price, ts)
            if filled:
                self._stops.pop(symbol, None)
                self._log(ts, f"SELL {pos.quantity:.0f} {symbol} @ {current_price:.4f}")

    def _check_stops(self, symbol: str, bar, ts: datetime) -> None:
        stop = self._stops.get(symbol)
        if stop is None:
            return

        triggered = False
        if stop.direction == "BUY" and bar.low <= stop.stop_price:
            triggered = True
            exit_price = stop.stop_price
        elif stop.direction == "SELL" and bar.high >= stop.stop_price:
            triggered = True
            exit_price = stop.stop_price
        else:
            return

        if triggered and symbol in self.portfolio.positions:
            pos = self.portfolio.positions[symbol]
            self.portfolio.sell(symbol, pos.quantity, exit_price, ts)
            self._stops.pop(symbol, None)
            self._log(ts, f"STOP {symbol} @ {exit_price:.4f} (stop hit)")

    def _log(self, ts: datetime, msg: str) -> None:
        entry = f"[{ts.date()}] {msg}"
        self.log.append(entry)
