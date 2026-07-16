"""Host-side ports of the Daisy Seed pinger-detection firmwares.

Copied verbatim from references/robosub-ee/research/tdoa (the EE team's
Orin-side ports) so the `pinger` node can run the same front/back verdict
against the live 96 kHz hydrophone stream. Only the intra-package imports
were made relative; the detection algorithm is unchanged.
"""
