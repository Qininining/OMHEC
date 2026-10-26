function cfg = default_config()
%DEFAULT_CONFIG Numerical defaults; length unit is millimetres.
cfg.observabilityInitializationSteps = 6;
cfg.processRotationStd = 0;
cfg.processTranslationStd = 0;
cfg.decayGain = 0.03;
cfg.decayCharacteristicLength = 0.10 / deg2rad(1);
cfg.jfgdSvdRelativeThreshold = 0.05;
end
