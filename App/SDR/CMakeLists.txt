target_sources(App PRIVATE
    sdr_spectrum.c
    sdr_resample.c
    sdr_iir.c
    sdr_iir_decim.c
    sdr_halfband_decim.c
    sdr_sideband.c
    sdr_ssb.c
    sdr_cw.c
    sdr_am.c
    sdr_fm_disc.c
    sdr_ctcss.c
    sdr_nbfm.c
    sdr_wfm.c
    sdr_agc.c
    sdr_squelch.c
    sdr_nr.c
    sdr_audio_resample.c
    sdr_tx.c
    sdr_rx.c
    sdr_compressor.c
    sdr_soft_clipper.c
    sdr_notch.c
    sdr_nb.c
)

target_include_directories(App PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
