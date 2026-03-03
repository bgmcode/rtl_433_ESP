"""
Backtesting engine and performance metrics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

from .data import MarketData
from .engine import TradingEngine
from .portfolio import Portfolio
from .risk import RiskManager, RiskParams
from .strategies import BaseStrategy


# ---------------------------------------------------------------------------
# Performance metrics
# ---------------------------------------------------------------------------

@dataclass
class PerformanceReport:
    strategy_name: str
    symbol: str
    start_date: datetime
    end_date: datetime
    initial_capital: float
    final_equity: float
    total_return_pct: float
    annualised_return_pct: float
    max_drawdown_pct: float
    sharpe_ratio: float
    sortino_ratio: float
    win_rate_pct: float
    profit_factor: float
    total_trades: int
    winning_trades: int
    losing_trades: int
    avg_win: float
    avg_loss: float
    largest_win: float
    largest_loss: float
    trade_log: List[str] = field(default_factory=list)

    def __str__(self) -> str:
        sep = "-" * 52
        lines = [
            sep,
            f"  Strategy : {self.strategy_name}",
            f"  Symbol   : {self.symbol}",
            f"  Period   : {self.start_date.date()} → {self.end_date.date()}",
            sep,
            f"  Initial Capital     : ${self.initial_capital:>12,.2f}",
            f"  Final Equity        : ${self.final_equity:>12,.2f}",
            f"  Total Return        : {self.total_return_pct:>+10.2f} %",
            f"  Ann. Return         : {self.annualised_return_pct:>+10.2f} %",
            f"  Max Drawdown        : {self.max_drawdown_pct:>10.2f} %",
            sep,
            f"  Sharpe Ratio        : {self.sharpe_ratio:>10.4f}",
            f"  Sortino Ratio       : {self.sortino_ratio:>10.4f}",
            sep,
            f"  Total Trades        : {self.total_trades:>10d}",
            f"  Win Rate            : {self.win_rate_pct:>10.2f} %",
            f"  Profit Factor       : {self.profit_factor:>10.4f}",
            f"  Avg Win / Avg Loss  : ${self.avg_win:>8,.2f} / ${self.avg_loss:>8,.2f}",
            f"  Largest Win / Loss  : ${self.largest_win:>8,.2f} / ${self.largest_loss:>8,.2f}",
            sep,
        ]
        return "\n".join(lines)


def _daily_returns(equity_curve: List[Tuple[datetime, float]]) -> List[float]:
    """Convert equity curve to percentage daily returns."""
    returns = []
    for i in range(1, len(equity_curve)):
        prev = equity_curve[i - 1][1]
        curr = equity_curve[i][1]
        if prev > 0:
            returns.append((curr - prev) / prev)
    return returns


def _sharpe(returns: List[float], risk_free: float = 0.0, periods: int = 252) -> float:
    n = len(returns)
    if n < 2:
        return 0.0
    mean = sum(returns) / n - risk_free / periods
    variance = sum((r - mean) ** 2 for r in returns) / (n - 1)
    std = math.sqrt(variance)
    if std < 1e-12:
        return 0.0
    return (mean / std) * math.sqrt(periods)


def _sortino(returns: List[float], risk_free: float = 0.0, periods: int = 252) -> float:
    n = len(returns)
    if n < 2:
        return 0.0
    mean = sum(returns) / n - risk_free / periods
    downside_sq = [r**2 for r in returns if r < 0]
    if not downside_sq:
        return float("inf")
    downside_std = math.sqrt(sum(downside_sq) / len(downside_sq))
    if downside_std < 1e-12:
        return 0.0
    return (mean / downside_std) * math.sqrt(periods)


def _max_drawdown(equity_curve: List[Tuple[datetime, float]]) -> float:
    """Returns max drawdown as a positive percentage (0-100)."""
    peak = -math.inf
    max_dd = 0.0
    for _, eq in equity_curve:
        if eq > peak:
            peak = eq
        dd = (peak - eq) / peak if peak > 0 else 0.0
        if dd > max_dd:
            max_dd = dd
    return max_dd * 100.0


def _trade_stats(
    trades: list,
    initial_capital: float,
) -> dict:
    pnls = []
    # Pair buys with subsequent sells
    buy_cost: Dict[str, Tuple[float, float]] = {}
    for t in trades:
        if t.direction == "BUY":
            buy_cost[t.symbol] = (t.price, t.quantity)
        elif t.direction == "SELL" and t.symbol in buy_cost:
            entry_price, qty = buy_cost.pop(t.symbol)
            pnl = (t.price - entry_price) * t.quantity - t.commission
            pnls.append(pnl)

    wins = [p for p in pnls if p > 0]
    losses = [p for p in pnls if p <= 0]

    total = len(pnls)
    win_rate = (len(wins) / total * 100) if total > 0 else 0.0
    avg_win = (sum(wins) / len(wins)) if wins else 0.0
    avg_loss = (sum(losses) / len(losses)) if losses else 0.0
    largest_win = max(wins) if wins else 0.0
    largest_loss = min(losses) if losses else 0.0
    gross_profit = sum(wins)
    gross_loss = abs(sum(losses))
    profit_factor = (gross_profit / gross_loss) if gross_loss > 0 else float("inf")

    return {
        "total": total,
        "wins": len(wins),
        "losses": len(losses),
        "win_rate": win_rate,
        "avg_win": avg_win,
        "avg_loss": abs(avg_loss),
        "largest_win": largest_win,
        "largest_loss": abs(largest_loss),
        "profit_factor": profit_factor,
    }


# ---------------------------------------------------------------------------
# Backtester
# ---------------------------------------------------------------------------

class Backtester:
    """
    Runs a full historical backtest for one strategy on one symbol.

    Usage::

        backtester = Backtester(initial_capital=100_000)
        report = backtester.run(strategy, market_data)
        print(report)
    """

    def __init__(
        self,
        initial_capital: float = 100_000.0,
        commission_rate: float = 0.001,
        risk_params: Optional[RiskParams] = None,
    ) -> None:
        self.initial_capital = initial_capital
        self.commission_rate = commission_rate
        self.risk_params = risk_params or RiskParams()

    def run(self, strategy: BaseStrategy, market_data: MarketData) -> PerformanceReport:
        portfolio = Portfolio(self.initial_capital, self.commission_rate)
        risk_mgr = RiskManager(self.risk_params)
        engine = TradingEngine(strategy, portfolio, risk_mgr)

        engine.run(market_data)

        equity_curve = portfolio.equity_curve
        bars = market_data.bars

        final_equity = equity_curve[-1][1] if equity_curve else self.initial_capital
        total_return = (final_equity - self.initial_capital) / self.initial_capital * 100

        # Annualise return
        days = (bars[-1].timestamp - bars[0].timestamp).days or 1
        ann_return = ((1 + total_return / 100) ** (365 / days) - 1) * 100

        daily_rets = _daily_returns(equity_curve)
        sharpe = _sharpe(daily_rets)
        sortino = _sortino(daily_rets)
        max_dd = _max_drawdown(equity_curve)

        stats = _trade_stats(portfolio.trades, self.initial_capital)

        return PerformanceReport(
            strategy_name=strategy.name,
            symbol=market_data.symbol,
            start_date=bars[0].timestamp,
            end_date=bars[-1].timestamp,
            initial_capital=self.initial_capital,
            final_equity=final_equity,
            total_return_pct=total_return,
            annualised_return_pct=ann_return,
            max_drawdown_pct=max_dd,
            sharpe_ratio=sharpe,
            sortino_ratio=sortino,
            win_rate_pct=stats["win_rate"],
            profit_factor=stats["profit_factor"],
            total_trades=stats["total"],
            winning_trades=stats["wins"],
            losing_trades=stats["losses"],
            avg_win=stats["avg_win"],
            avg_loss=stats["avg_loss"],
            largest_win=stats["largest_win"],
            largest_loss=stats["largest_loss"],
            trade_log=engine.log,
        )
