package swarmdrone.decentralizedswarmcoverage_ijrcs;

/**
 * Immutable integer grid coordinate (row, column).
 */
public record Point(int row, int col) {

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Point other)) return false;
        return row == other.row && col == other.col;
    }

}
