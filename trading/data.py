"""
Market data module for the algorithmic trading system.
Handles fetching, caching, and serving OHLCV price data.
"""

from __future__ import annotations

import csv
import io
import random
import math
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Dict, List, Optional


@dataclass
class Bar:
    """A single OHLCV price bar."""
    timestamp: datetime
    open: float
    high: float
    low: float
    close: float
    volume: float

    @property
    def typical_price(self) -> float:
        return (self.high + self.low + self.close) / 3.0


@dataclass
class MarketData:
    """Container for a symbol's historical bars."""
    symbol: str
    bars: List[Bar] = field(default_factory=list)

    def closes(self) -> List[float]:
        return [b.close for b in self.bars]

    def highs(self) -> List[float]:
        return [b.high for b in self.bars]

    def lows(self) -> List[float]:
        return [b.low for b in self.bars]

    def volumes(self) -> List[float]:
        return [b.volume for b in self.bars]

    def timestamps(self) -> List[datetime]:
        return [b.timestamp for b in self.bars]

    def slice(self, start: int, end: int) -> "MarketData":
        md = MarketData(symbol=self.symbol)
        md.bars = self.bars[start:end]
        return md


def _sma(values: List[float], period: int) -> List[Optional[float]]:
    result: List[Optional[float]] = [None] * (period - 1)
    for i in range(period - 1, len(values)):
        result.append(sum(values[i - period + 1 : i + 1]) / period)
    return result


def generate_synthetic_data(
    symbol: str,
    start: datetime,
    days: int,
    start_price: float = 100.0,
    volatility: float = 0.015,
    drift: float = 0.0002,
    seed: Optional[int] = 42,
) -> MarketData:
    """
    Generate synthetic OHLCV price data using a geometric Brownian motion model.

    Args:
        symbol:       Ticker symbol name (for labelling).
        start:        Start date of the series.
        days:         Number of trading days to generate.
        start_price:  Initial closing price.
        volatility:   Daily return standard deviation.
        drift:        Daily mean return.
        seed:         Random seed for reproducibility.

    Returns:
        A MarketData object populated with synthetic bars.
    """
    rng = random.Random(seed)
    md = MarketData(symbol=symbol)

    price = start_price
    current = start

    for _ in range(days):
        # Skip weekends (simple approximation)
        while current.weekday() >= 5:
            current += timedelta(days=1)

        # GBM daily return
        z = rng.gauss(0, 1)
        daily_return = drift + volatility * z
        close = price * math.exp(daily_return)

        # Intraday range
        intraday_vol = volatility * rng.uniform(0.5, 1.5)
        high = close * math.exp(abs(rng.gauss(0, intraday_vol)))
        low = close * math.exp(-abs(rng.gauss(0, intraday_vol)))
        open_price = price * math.exp(rng.gauss(0, intraday_vol * 0.5))

        # Ensure OHLC consistency
        high = max(high, open_price, close)
        low = min(low, open_price, close)

        volume = rng.uniform(500_000, 2_000_000) * (1 + abs(daily_return) * 10)

        md.bars.append(
            Bar(
                timestamp=current,
                open=round(open_price, 4),
                high=round(high, 4),
                low=round(low, 4),
                close=round(close, 4),
                volume=round(volume, 0),
            )
        )

        price = close
        current += timedelta(days=1)

    return md


def load_csv(symbol: str, path: str) -> MarketData:
    """
    Load OHLCV data from a CSV file.

    Expected columns (case-insensitive):
        date/timestamp, open, high, low, close, volume

    Returns:
        MarketData populated from the CSV.
    """
    md = MarketData(symbol=symbol)
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        headers = {k.lower().strip(): k for k in (reader.fieldnames or [])}

        date_col = headers.get("date") or headers.get("timestamp")
        if not date_col:
            raise ValueError("CSV must have a 'date' or 'timestamp' column.")

        for row in reader:
            ts_str = row[date_col].strip()
            for fmt in ("%Y-%m-%d", "%Y-%m-%d %H:%M:%S", "%m/%d/%Y"):
                try:
                    ts = datetime.strptime(ts_str, fmt)
                    break
                except ValueError:
                    pass
            else:
                raise ValueError(f"Cannot parse date: {ts_str}")

            md.bars.append(
                Bar(
                    timestamp=ts,
                    open=float(row[headers["open"]]),
                    high=float(row[headers["high"]]),
                    low=float(row[headers["low"]]),
                    close=float(row[headers["close"]]),
                    volume=float(row.get(headers.get("volume", ""), 0) or 0),
                )
            )

    md.bars.sort(key=lambda b: b.timestamp)
    return md
