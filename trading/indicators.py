"""
Technical indicators used by trading strategies.
All functions operate on plain Python lists to avoid external dependencies.
"""

from __future__ import annotations

from typing import List, Optional, Tuple


def sma(values: List[float], period: int) -> List[Optional[float]]:
    """Simple Moving Average."""
    out: List[Optional[float]] = [None] * (period - 1)
    for i in range(period - 1, len(values)):
        out.append(sum(values[i - period + 1 : i + 1]) / period)
    return out


def ema(values: List[float], period: int) -> List[Optional[float]]:
    """Exponential Moving Average."""
    if not values:
        return []
    k = 2.0 / (period + 1)
    out: List[Optional[float]] = [None] * (period - 1)
    if len(values) < period:
        return [None] * len(values)
    initial = sum(values[:period]) / period
    out.append(initial)
    current = initial
    for v in values[period:]:
        current = v * k + current * (1 - k)
        out.append(current)
    return out


def rsi(values: List[float], period: int = 14) -> List[Optional[float]]:
    """Relative Strength Index (Wilder smoothing)."""
    out: List[Optional[float]] = [None] * period
    if len(values) <= period:
        return [None] * len(values)

    gains = []
    losses = []
    for i in range(1, period + 1):
        diff = values[i] - values[i - 1]
        gains.append(max(diff, 0))
        losses.append(max(-diff, 0))

    avg_gain = sum(gains) / period
    avg_loss = sum(losses) / period

    def _rsi_value(ag: float, al: float) -> float:
        if al == 0:
            return 100.0
        rs = ag / al
        return 100.0 - (100.0 / (1 + rs))

    out.append(_rsi_value(avg_gain, avg_loss))

    for i in range(period + 1, len(values)):
        diff = values[i] - values[i - 1]
        gain = max(diff, 0)
        loss = max(-diff, 0)
        avg_gain = (avg_gain * (period - 1) + gain) / period
        avg_loss = (avg_loss * (period - 1) + loss) / period
        out.append(_rsi_value(avg_gain, avg_loss))

    return out


def bollinger_bands(
    values: List[float], period: int = 20, num_std: float = 2.0
) -> Tuple[List[Optional[float]], List[Optional[float]], List[Optional[float]]]:
    """
    Bollinger Bands.

    Returns:
        (upper, middle, lower) each as a list aligned to `values`.
    """
    middle = sma(values, period)
    upper: List[Optional[float]] = []
    lower: List[Optional[float]] = []

    for i, mid in enumerate(middle):
        if mid is None:
            upper.append(None)
            lower.append(None)
        else:
            window = values[i - period + 1 : i + 1]
            mean = mid
            std = (sum((x - mean) ** 2 for x in window) / period) ** 0.5
            upper.append(mid + num_std * std)
            lower.append(mid - num_std * std)

    return upper, middle, lower


def macd(
    values: List[float],
    fast: int = 12,
    slow: int = 26,
    signal: int = 9,
) -> Tuple[List[Optional[float]], List[Optional[float]], List[Optional[float]]]:
    """
    MACD (Moving Average Convergence/Divergence).

    Returns:
        (macd_line, signal_line, histogram)
    """
    fast_ema = ema(values, fast)
    slow_ema = ema(values, slow)

    macd_line: List[Optional[float]] = []
    for f, s in zip(fast_ema, slow_ema):
        if f is None or s is None:
            macd_line.append(None)
        else:
            macd_line.append(f - s)

    # Signal line: EMA of the non-None portion of macd_line
    valid_start = next((i for i, v in enumerate(macd_line) if v is not None), None)
    signal_line: List[Optional[float]] = [None] * len(macd_line)
    histogram: List[Optional[float]] = [None] * len(macd_line)

    if valid_start is not None:
        valid_macd = [v for v in macd_line if v is not None]
        valid_signal = ema(valid_macd, signal)  # type: ignore[arg-type]
        offset = valid_start
        for j, sv in enumerate(valid_signal):
            signal_line[offset + j] = sv
            ml = macd_line[offset + j]
            if sv is not None and ml is not None:
                histogram[offset + j] = ml - sv

    return macd_line, signal_line, histogram


def atr(
    highs: List[float],
    lows: List[float],
    closes: List[float],
    period: int = 14,
) -> List[Optional[float]]:
    """Average True Range."""
    if len(highs) != len(lows) or len(highs) != len(closes):
        raise ValueError("highs, lows, closes must have equal length")

    trs: List[float] = []
    for i in range(len(highs)):
        if i == 0:
            trs.append(highs[i] - lows[i])
        else:
            trs.append(
                max(
                    highs[i] - lows[i],
                    abs(highs[i] - closes[i - 1]),
                    abs(lows[i] - closes[i - 1]),
                )
            )

    out: List[Optional[float]] = [None] * (period - 1)
    if len(trs) < period:
        return [None] * len(trs)

    current = sum(trs[:period]) / period
    out.append(current)
    for tr in trs[period:]:
        current = (current * (period - 1) + tr) / period
        out.append(current)

    return out
