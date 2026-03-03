"""
Risk management: position sizing and trade filtering.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class RiskParams:
    """
    Central configuration for all risk controls.

    Attributes:
        max_position_pct:   Maximum portfolio percentage in a single position.
        risk_per_trade_pct: Maximum portfolio percentage risked on one trade
                            (used with stop-loss to size the position).
        stop_loss_atr_mult: Stop-loss = entry ± (ATR * stop_loss_atr_mult).
                            Set to None to disable ATR-based stops.
        fixed_stop_pct:     Alternative: fixed percentage stop from entry.
        max_open_positions: Hard cap on simultaneous open positions.
        max_drawdown_pct:   Halt trading if portfolio drawdown exceeds this.
    """

    max_position_pct: float = 0.20        # 20 % of equity per trade
    risk_per_trade_pct: float = 0.01      # Risk 1 % per trade
    stop_loss_atr_mult: Optional[float] = 2.0
    fixed_stop_pct: float = 0.03          # 3 % fixed stop (fallback)
    max_open_positions: int = 5
    max_drawdown_pct: float = 0.20        # 20 % drawdown halt


class RiskManager:
    """
    Evaluates and enforces risk limits for each trade signal.
    """

    def __init__(self, params: Optional[RiskParams] = None) -> None:
        self.params = params or RiskParams()
        self._peak_equity: Optional[float] = None
        self._trading_halted: bool = False

    # ------------------------------------------------------------------
    # Drawdown monitoring
    # ------------------------------------------------------------------

    def update_equity(self, equity: float) -> None:
        """
        Call once per bar to update the high-water mark and check drawdown.
        """
        if self._peak_equity is None or equity > self._peak_equity:
            self._peak_equity = equity

        drawdown = (self._peak_equity - equity) / self._peak_equity
        if drawdown >= self.params.max_drawdown_pct:
            self._trading_halted = True
        else:
            self._trading_halted = False

    @property
    def trading_halted(self) -> bool:
        return self._trading_halted

    @property
    def current_drawdown(self) -> float:
        if self._peak_equity is None or self._peak_equity == 0:
            return 0.0
        # This is recalculated at query time; use update_equity to track.
        return 0.0

    # ------------------------------------------------------------------
    # Position sizing
    # ------------------------------------------------------------------

    def position_size(
        self,
        entry_price: float,
        equity: float,
        atr: Optional[float] = None,
    ) -> float:
        """
        Calculate the number of units to trade.

        Uses ATR-based stop if `atr` is provided and stop_loss_atr_mult is
        set; otherwise falls back to the fixed percentage stop.

        Returns the number of units (may be fractional; caller should round).
        """
        risk_amount = equity * self.params.risk_per_trade_pct

        if atr is not None and self.params.stop_loss_atr_mult is not None:
            stop_distance = atr * self.params.stop_loss_atr_mult
        else:
            stop_distance = entry_price * self.params.fixed_stop_pct

        if stop_distance < 1e-9:
            return 0.0

        units_by_risk = risk_amount / stop_distance
        units_by_cap = (equity * self.params.max_position_pct) / entry_price
        return min(units_by_risk, units_by_cap)

    def stop_price(
        self,
        entry_price: float,
        direction: str,
        atr: Optional[float] = None,
    ) -> float:
        """
        Calculate the stop-loss price for a new position.

        Args:
            entry_price: Fill price.
            direction:   'BUY' or 'SELL'.
            atr:         Current ATR value; None to use fixed percentage.

        Returns:
            Stop-loss price.
        """
        if atr is not None and self.params.stop_loss_atr_mult is not None:
            distance = atr * self.params.stop_loss_atr_mult
        else:
            distance = entry_price * self.params.fixed_stop_pct

        if direction == "BUY":
            return entry_price - distance
        return entry_price + distance

    # ------------------------------------------------------------------
    # Signal filtering
    # ------------------------------------------------------------------

    def allow_trade(self, open_positions: int) -> bool:
        """
        Returns False if the trade should be blocked (halted or max positions).
        """
        if self._trading_halted:
            return False
        if open_positions >= self.params.max_open_positions:
            return False
        return True
