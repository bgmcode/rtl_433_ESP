"""
Portfolio management: tracks cash, positions, and equity curve.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple


@dataclass
class Position:
    symbol: str
    quantity: float       # number of shares / units (positive = long)
    avg_cost: float       # average cost per unit
    opened_at: datetime

    @property
    def market_value(self) -> float:
        return self.quantity * self.avg_cost

    def unrealised_pnl(self, current_price: float) -> float:
        return self.quantity * (current_price - self.avg_cost)


@dataclass
class Trade:
    symbol: str
    direction: str        # "BUY" or "SELL"
    quantity: float
    price: float
    timestamp: datetime
    commission: float = 0.0

    @property
    def value(self) -> float:
        return self.quantity * self.price


class Portfolio:
    """
    Tracks cash, open positions, and realised P&L.

    All prices are in the same currency as the initial capital.
    """

    def __init__(
        self,
        initial_capital: float = 100_000.0,
        commission_rate: float = 0.001,  # 0.1 % per trade
    ) -> None:
        self.initial_capital = initial_capital
        self.cash = initial_capital
        self.commission_rate = commission_rate

        self._positions: Dict[str, Position] = {}
        self._trades: List[Trade] = []
        self._equity_curve: List[Tuple[datetime, float]] = []
        self.realised_pnl: float = 0.0

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def positions(self) -> Dict[str, Position]:
        return dict(self._positions)

    @property
    def trades(self) -> List[Trade]:
        return list(self._trades)

    @property
    def equity_curve(self) -> List[Tuple[datetime, float]]:
        return list(self._equity_curve)

    def market_value(self, prices: Dict[str, float]) -> float:
        """Total portfolio value (cash + open positions at current prices)."""
        mv = self.cash
        for symbol, pos in self._positions.items():
            price = prices.get(symbol, pos.avg_cost)
            mv += pos.quantity * price
        return mv

    def equity(self, prices: Dict[str, float]) -> float:
        return self.market_value(prices)

    # ------------------------------------------------------------------
    # Order execution
    # ------------------------------------------------------------------

    def buy(
        self,
        symbol: str,
        quantity: float,
        price: float,
        timestamp: datetime,
    ) -> bool:
        """
        Execute a market buy order.

        Returns True if the order was filled, False if insufficient cash.
        """
        commission = price * quantity * self.commission_rate
        total_cost = price * quantity + commission

        if total_cost > self.cash:
            return False

        self.cash -= total_cost

        if symbol in self._positions:
            pos = self._positions[symbol]
            new_qty = pos.quantity + quantity
            pos.avg_cost = (pos.avg_cost * pos.quantity + price * quantity) / new_qty
            pos.quantity = new_qty
        else:
            self._positions[symbol] = Position(
                symbol=symbol,
                quantity=quantity,
                avg_cost=price,
                opened_at=timestamp,
            )

        self._trades.append(
            Trade(symbol, "BUY", quantity, price, timestamp, commission)
        )
        return True

    def sell(
        self,
        symbol: str,
        quantity: float,
        price: float,
        timestamp: datetime,
    ) -> bool:
        """
        Execute a market sell order.

        Returns True if the order was filled, False if no position exists.
        """
        if symbol not in self._positions:
            return False

        pos = self._positions[symbol]
        qty = min(quantity, pos.quantity)
        if qty <= 0:
            return False

        commission = price * qty * self.commission_rate
        proceeds = price * qty - commission
        self.cash += proceeds

        # Realised P&L
        self.realised_pnl += qty * (price - pos.avg_cost) - commission

        pos.quantity -= qty
        if pos.quantity <= 1e-9:
            del self._positions[symbol]

        self._trades.append(
            Trade(symbol, "SELL", qty, price, timestamp, commission)
        )
        return True

    def close_all(self, prices: Dict[str, float], timestamp: datetime) -> None:
        """Close every open position at the provided prices."""
        for symbol in list(self._positions.keys()):
            pos = self._positions[symbol]
            price = prices.get(symbol, pos.avg_cost)
            self.sell(symbol, pos.quantity, price, timestamp)

    def record_equity(self, timestamp: datetime, prices: Dict[str, float]) -> None:
        """Snapshot the current equity value for the equity curve."""
        self._equity_curve.append((timestamp, self.equity(prices)))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def position_size_by_risk(
        self,
        symbol: str,
        entry_price: float,
        stop_price: float,
        risk_pct: float = 0.01,
        current_equity: Optional[float] = None,
    ) -> float:
        """
        Calculate position size such that the loss from entry to stop equals
        `risk_pct` of the portfolio equity.

        Returns the number of units (shares) to buy (may be fractional).
        """
        equity = current_equity if current_equity is not None else self.cash
        risk_amount = equity * risk_pct
        risk_per_unit = abs(entry_price - stop_price)
        if risk_per_unit < 1e-9:
            return 0.0
        return risk_amount / risk_per_unit

    def summary(self, prices: Dict[str, float]) -> dict:
        total_equity = self.equity(prices)
        total_return = (total_equity - self.initial_capital) / self.initial_capital
        return {
            "initial_capital": self.initial_capital,
            "cash": self.cash,
            "total_equity": total_equity,
            "realised_pnl": self.realised_pnl,
            "total_return_pct": total_return * 100,
            "open_positions": len(self._positions),
            "total_trades": len(self._trades),
        }
