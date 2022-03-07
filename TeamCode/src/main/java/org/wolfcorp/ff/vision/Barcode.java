package org.wolfcorp.ff.vision;

public enum Barcode {
    /** Corresponds with the zero position of the slide encoder */
    ZERO,
    /** Corresponds with the excess-dumping position of the slide encoder. **/
    EXCESS,
    /** Corresponds with the bottom tier of the hub */
    BOT,
    /** Corresponds with the middle tier of the hub */
    MID,
    /** Corresponds with the top tier of the hub */
    TOP,
    /** Corresponds with the top tier of the hub when tipped */
    SUPERTOP,
    DIRTY
}
