import unittest

import collect_calibration as cc


class CalibrationCollectorTests(unittest.TestCase):
    def test_normalize_needle_accepts_common_spellings(self):
        self.assertEqual(cc.normalize_needle("2.0"), "2.00")
        self.assertEqual(cc.normalize_needle("2.00"), "2.00")
        self.assertEqual(cc.normalize_needle("0.5"), "0.50")

    def test_parse_evt_line_accepts_optional_rise_fall(self):
        full = cc.parse_evt_line("EVT,12,345.6,88,23,4,19")
        short = cc.parse_evt_line("EVT,13,400,90,25")

        self.assertEqual(full["drop_id"], "12")
        self.assertEqual(full["integral"], 345.6)
        self.assertEqual(full["rise"], "4")
        self.assertEqual(full["fall"], "19")
        self.assertEqual(short["drop_id"], "13")
        self.assertEqual(short["rise"], "")
        self.assertEqual(short["fall"], "")

    def test_parse_evt_line_v3_layout_with_vol_and_total(self):
        # v3固件：EVT,drop_id,integral,vol_0p01mm3,total_0p01mm3,peak,width,rise,fall
        evt = cc.parse_evt_line("EVT,4,2294623,3846,15384,3164,118,17,100")

        self.assertEqual(evt["drop_id"], "4")
        self.assertEqual(evt["integral"], 2294623.0)
        self.assertEqual(evt["peak"], "3164")
        self.assertEqual(evt["width"], "118")
        self.assertEqual(evt["rise"], "17")
        self.assertEqual(evt["fall"], "100")

    def test_parse_evt_line_ignores_non_evt_and_bad_evt(self):
        self.assertIsNone(cc.parse_evt_line("BOOT,ready"))
        self.assertIsNone(cc.parse_evt_line("EVT,missing,fields"))

    def test_finalize_stats_discards_first_five_and_filters_outliers(self):
        samples = [
            {"integral": value}
            for value in [1, 2, 3, 4, 5, 100, 102, 98, 101, 100, 250]
        ]

        stats = cc.calculate_summary_stats(samples)

        self.assertEqual(stats["collected_drops"], 11)
        self.assertEqual(stats["excluded_first_drops"], 5)
        self.assertEqual(stats["valid_drops"], 5)
        self.assertEqual(stats["outlier_drops"], 1)
        self.assertAlmostEqual(stats["integral_median_valid"], 100.0)


if __name__ == "__main__":
    unittest.main()
