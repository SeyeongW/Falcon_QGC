pragma Singleton

import QtQuick

import QGroundControl
import QGroundControl.Controls

/// Falcon GCS design tokens — IBM Carbon g100, black + light-sky.
///
/// Values are taken from Carbon's own sources rather than eyeballed:
/// `packages/themes/src/dtcg/g100.json` for the token roles and
/// `packages/colors/src/colors.ts` for the palette they alias.
/// Carbon is Apache-2.0; only the values are used here, not its components.
///
/// Carbon conventions kept as-is:
///
///  * **Square corners.** Carbon has no border radius. It is the most
///    recognisable thing about it and the fastest way out of the rounded-card
///    web look.
///  * **The layer model.** Depth is a stack of opaque fills, not shadows or
///    translucency, each a fixed step up the gray ramp.
///  * **Neutral borders only.** Carbon separates with `border-subtle`; colour is
///    reserved for meaning.
///  * **The 8px spacing scale** and a fixed type scale, so panels align to a
///    common grid instead of ad-hoc font-width fractions.
///
/// Two deliberate deviations from stock g100:
///
///  1. The background drops from Carbon's gray-100 (#161616) to **true black**,
///     and every layer shifts one step down the ramp to suit. On an OLED-ish
///     ops display this reads as a deeper instrument surface, and it buys one
///     extra distinguishable layer at the dark end.
///  2. `interactive` is Carbon **cyan-30 (#82cfff)** — the light-sky accent —
///     in place of blue-50. Cyan-30 is a "light" step, so it stays legible on
///     black without the glare of a saturated mid-tone.
QtObject {
    id: theme

    // --- Carbon layer model, shifted one step darker -----------------------
    readonly property color base:      "#000000"   // black.default
    readonly property color surface1:  "#161616"   // gray-100
    readonly property color surface2:  "#262626"   // gray-90
    readonly property color surface3:  "#393939"   // gray-80  (hover / selected)
    readonly property color sunken:    "#000000"   // video + 3D wells

    // border-subtle / border-strong.
    readonly property color hairline:       "#393939"   // gray-80
    readonly property color hairlineStrong: "#525252"   // gray-70

    // --- text (Carbon text-primary / secondary / helper) --------------------
    readonly property color textPrimary:   "#F4F4F4"   // gray-10
    readonly property color textSecondary: "#C6C6C6"   // gray-30
    readonly property color textMuted:     "#A8A8A8"   // gray-40

    // --- meaning-bearing colour --------------------------------------------
    readonly property color accent:     "#82CFFF"   // cyan-30, the light-sky accent
    readonly property color accentDeep: "#33B1FF"   // cyan-40, for fills behind text
    readonly property color ok:         "#42BE65"   // support-success (green-40)
    readonly property color caution:    "#F1C21B"   // support-warning (yellow-30)
    readonly property color warn:       "#FA4D56"   // support-error   (red-50)

    /// Text colour to place on top of an `accent`/`accentDeep` fill. Carbon's
    /// light steps are bright enough that the readable pairing is the dark
    /// background, not white.
    readonly property color textOnAccent: "#000000"

    /// Status colour by severity rank: 0 ok, 1 caution, 2 warn.
    function statusColor(rank) {
        switch (rank) {
        case 2:  return warn
        case 1:  return caution
        default: return ok
        }
    }

    /// Width of the left-edge bar that marks the panel the operator should be
    /// looking at. Carbon uses exactly this device for selected rows; borrowing
    /// it gives hierarchy without drawing another box.
    readonly property real activeBarWidth: 3

    /// Thickness of the rule that heads a panel. One rule beats four borders:
    /// it separates without enclosing, so panels stop reading as equal tiles.
    readonly property real ruleWidth: 1

    // --- geometry -----------------------------------------------------------
    // Carbon is square. Both stay at 0; they remain separate tokens so a future
    // deviation does not have to be hunted down in every file.
    readonly property real radiusPanel:   0
    readonly property real radiusControl: 0

    // Carbon spacing scale (spacing-01 .. spacing-05).
    readonly property real space1: 2
    readonly property real space2: 8
    readonly property real space3: 12
    readonly property real space4: 16
    readonly property real space5: 24

    // --- type ---------------------------------------------------------------
    // Carbon's productive type scale ratios, expressed against ScreenTools so
    // the app still honours the user's UI scaling.
    readonly property real fontCaption: ScreenTools.defaultFontPixelHeight * 0.70   // label-01
    readonly property real fontBody:    ScreenTools.defaultFontPixelHeight * 0.85   // body-compact-01
    readonly property real fontValue:   ScreenTools.defaultFontPixelHeight * 1.15   // heading-03
    readonly property real fontReadout: ScreenTools.defaultFontPixelHeight * 1.65   // heading-04

    // --- fonts ---------------------------------------------------------------
    // Registered in QGCApplication::init() from qgcresources.qrc.
    readonly property string fontFamily:     "IBM Plex Sans"
    /// Telemetry readouts. A fixed digit advance is what stops a changing value
    /// from nudging its neighbours, which is most of why a number reads as an
    /// instrument rather than as body text.
    readonly property string fontFamilyMono: "IBM Plex Mono"

    /// Kept for text that is not set in Plex Mono but still shows digits; a
    /// no-op where the family already has fixed-width figures.
    readonly property var numericFeatures: ({ "tnum": 1 })

    /// Carbon label-01 tracking.
    readonly property real captionLetterSpacing: 0.32
}
