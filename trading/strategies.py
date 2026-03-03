"""
Trading strategies.

Each strategy implements:
    generate_signals(market_data) -> List[Signal]

A Signal carries a timestamp, direction (BUY / SELL / FLAT), and
the close price at the time the signal was generated.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

from .data import MarketData
from . import indicators


class Direction(Enum):
    BUY = "BUY"
    SELL = "SELL"
    FLAT = "FLAT"


@dataclass
class Signal:
    index: int          # Bar index that triggered the signal
    direction: Direction
    price: float        # Entry price (close of triggering bar)
    strategy_name: str
    metadata: dict = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.metadata is None:
            self.metadata = {}


class BaseStrategy:
    """Abstract base for all strategies."""

    name: str = "BaseStrategy"

    def generate_signals(self, market_data: MarketData) -> List[Signal]:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Moving-Average Crossover
# ---------------------------------------------------------------------------

class MACrossoverStrategy(BaseStrategy):
    """
    Classic dual moving-average crossover strategy.

    BUY  when the fast SMA crosses above the slow SMA.
    SELL when the fast SMA crosses below the slow SMA.
    """

    name = "MA_Crossover"

    def __init__(self, fast: int = 20, slow: int = 50) -> None:
        self.fast = fast
        self.slow = slow
        self.name = f"MA_Crossover({fast},{slow})"

    def generate_signals(self, market_data: MarketData) -> List[Signal]:
        closes = market_data.closes()
        fast_ma = indicators.sma(closes, self.fast)
        slow_ma = indicators.sma(closes, self.slow)

        signals: List[Signal] = []
        for i in range(1, len(closes)):
            f_prev, f_curr = fast_ma[i - 1], fast_ma[i]
            s_prev, s_curr = slow_ma[i - 1], slow_ma[i]

            if any(v is None for v in (f_prev, f_curr, s_prev, s_curr)):
                continue

            # Golden cross
            if f_prev <= s_prev and f_curr > s_curr:  # type: ignore[operator]
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.BUY,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"fast_ma": f_curr, "slow_ma": s_curr},
                    )
                )
            # Death cross
            elif f_prev >= s_prev and f_curr < s_curr:  # type: ignore[operator]
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.SELL,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"fast_ma": f_curr, "slow_ma": s_curr},
                    )
                )

        return signals


# ---------------------------------------------------------------------------
# RSI Strategy
# ---------------------------------------------------------------------------

class RSIStrategy(BaseStrategy):
    """
    RSI mean-reversion strategy.

    BUY  when RSI crosses up through the oversold level.
    SELL when RSI crosses down through the overbought level.
    """

    name = "RSI"

    def __init__(
        self,
        period: int = 14,
        oversold: float = 30.0,
        overbought: float = 70.0,
    ) -> None:
        self.period = period
        self.oversold = oversold
        self.overbought = overbought
        self.name = f"RSI({period},{oversold},{overbought})"

    def generate_signals(self, market_data: MarketData) -> List[Signal]:
        closes = market_data.closes()
        rsi_vals = indicators.rsi(closes, self.period)

        signals: List[Signal] = []
        for i in range(1, len(closes)):
            r_prev, r_curr = rsi_vals[i - 1], rsi_vals[i]
            if r_prev is None or r_curr is None:
                continue

            if r_prev <= self.oversold < r_curr:
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.BUY,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"rsi": r_curr},
                    )
                )
            elif r_prev >= self.overbought > r_curr:
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.SELL,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"rsi": r_curr},
                    )
                )

        return signals


# ---------------------------------------------------------------------------
# Bollinger Band Mean-Reversion
# ---------------------------------------------------------------------------

class BollingerBandStrategy(BaseStrategy):
    """
    Bollinger Band mean-reversion strategy.

    BUY  when price touches / crosses below the lower band (oversold).
    SELL when price touches / crosses above the upper band (overbought).
    """

    name = "BollingerBand"

    def __init__(self, period: int = 20, num_std: float = 2.0) -> None:
        self.period = period
        self.num_std = num_std
        self.name = f"BollingerBand({period},{num_std})"

    def generate_signals(self, market_data: MarketData) -> List[Signal]:
        closes = market_data.closes()
        upper, middle, lower = indicators.bollinger_bands(closes, self.period, self.num_std)

        signals: List[Signal] = []
        for i in range(1, len(closes)):
            u, l = upper[i], lower[i]
            if u is None or l is None:
                continue

            c_prev, c_curr = closes[i - 1], closes[i]
            l_prev = lower[i - 1]
            u_prev = upper[i - 1]

            if l_prev is not None and c_prev > l_prev and c_curr <= l:  # type: ignore[operator]
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.BUY,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"upper": u, "lower": l, "middle": middle[i]},
                    )
                )
            elif u_prev is not None and c_prev < u_prev and c_curr >= u:  # type: ignore[operator]
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.SELL,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"upper": u, "lower": l, "middle": middle[i]},
                    )
                )

        return signals


# ---------------------------------------------------------------------------
# MACD Strategy
# ---------------------------------------------------------------------------

class MACDStrategy(BaseStrategy):
    """
    MACD histogram sign-change strategy.

    BUY  when MACD histogram crosses from negative to positive.
    SELL when MACD histogram crosses from positive to negative.
    """

    name = "MACD"

    def __init__(self, fast: int = 12, slow: int = 26, signal: int = 9) -> None:
        self.fast = fast
        self.slow = slow
        self.signal = signal
        self.name = f"MACD({fast},{slow},{signal})"

    def generate_signals(self, market_data: MarketData) -> List[Signal]:
        closes = market_data.closes()
        _, _, histogram = indicators.macd(closes, self.fast, self.slow, self.signal)

        signals: List[Signal] = []
        for i in range(1, len(closes)):
            h_prev, h_curr = histogram[i - 1], histogram[i]
            if h_prev is None or h_curr is None:
                continue

            if h_prev <= 0 < h_curr:
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.BUY,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"histogram": h_curr},
                    )
                )
            elif h_prev >= 0 > h_curr:
                signals.append(
                    Signal(
                        index=i,
                        direction=Direction.SELL,
                        price=closes[i],
                        strategy_name=self.name,
                        metadata={"histogram": h_curr},
                    )
                )

        return signals
