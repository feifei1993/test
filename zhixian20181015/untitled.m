clc
clear
close all
x=[0.140032 0.136395 0.123664 0.13003 0.128211 0.137304 0.135485 0.137304 0.130939 0.132757 0.131848 0.133667 0.134576 0.134576 0.12912 0.127302 0.123664 0.134576 0.12912 0.132757 0.130939 0.137304 0.14185 0.132757 0.136395 0.13003 0.130939 0.13003 0.134576 0.12912 0.133667 0.137304 0.13003 0.128211 0.12912 0.136395 0.130939 0.133667 0.134576 0.133667 0.12912 0.130939 0.13003 0.158218 0.269152 0.345533 0.373721 0.387361 1.93498 1.93135 1.9268 1.92498 1.92225 1.91771 1.92135 1.92498 1.92135 1.9168 1.91407 1.91225 1.91225 1.91134 1.90589 1.90316 1.90589 1.90498 1.90225 1.90134 1.89498 1.8977 1.89134 1.89134 1.89952 1.89407 1.89407 1.89861 1.89679 1.89679 1.89225 1.89225 1.89316 1.88952 1.88952 1.89498 1.89316 1.89225 1.89316 1.89316 1.88679 1.89407 1.89589 1.90134 1.89952 1.89952 1.89952 1.90225 1.90498 1.90134 1.90134 1.90316 1.90316 1.90589 1.90498 1.90498 1.91225 1.91589 1.91225 1.91407 1.91407 1.91589 1.91498 1.92498 1.9268 1.92498 1.92407 1.92407 1.92771 1.93589 1.93589 1.93862 1.93953 1.94044 1.94317 1.95226 1.9459 1.96408 1.96499 1.95772 1.96681 1.96499 1.96408 1.98954 2.01955 2.03046 2.03774 2.01682 1.99136 1.98409 1.97772 1.94408 1.85951 1.84315 1.82951 1.83405 1.83678 1.84133 1.84406 1.85406 1.85133 1.85679 1.86315 1.87224 1.87406 1.87952 1.88406 1.88952 1.89498 1.89952 1.90407 1.91134 1.91043 1.92135 1.92771 1.93135 1.93589 1.95681 2.08502 2.20686 2.21505 2.2205 2.22687 2.22869 2.23869 2.0818 2.08769 2.09526 2.10031 2.11125 2.11798 2.12387 2.12976 2.13818 2.1508 2.15837 2.16342 2.17436 2.18446 2.18698 2.19371 2.19708 2.17773 2.17689 2.19035 2.19792 2.20297 2.21475 2.22064 2.23242 2.23831 2.23495 2.22148 2.20465 2.18698 2.17352 2.15501 2.14239 2.1264 2.11209 2.10199 2.08264 2.07002 2.0574 2.04562 2.03131 2.01701 2.00438 1.99513 1.98503 1.9682 1.95642 1.94801 1.93707 1.92444 1.91014 1.90341 1.88826 1.88994 1.87648 1.86722 1.85965 1.85039 1.83946 1.82599 1.81589 1.80748 1.81085 1.79654 1.78055 1.77382 1.76541 1.76456 1.76204 1.74942 1.73848 1.73511 1.72165 1.71913 1.71155 1.70482 1.69977 1.68883 1.68631 1.67958 1.66948 1.66191 1.65686 1.65012 1.64508 1.64087 1.63245 1.62825 1.62236 1.62067 1.6131 1.60973 1.60132 1.59964 1.59627 1.58197 1.58197 1.57608 1.58281 1.56682 1.56261 1.56009 1.55756 1.5441 1.54915 1.54747 1.53989 1.53484 1.52979 1.53148 1.52559 1.51717 1.51381 1.51381 1.50707 1.51128 1.49866 1.50287 1.49361 1.49361 1.49109 1.48856 1.47931 1.48267 1.47426 1.47931 1.47594 1.46837 1.46584 1.46416 1.46248 1.46584 1.45911 1.45743 1.44901 1.45322 1.44901 1.44396 1.44312 1.44481 1.43555 1.43723 1.43639 1.44396 1.44481 1.43723 1.43218 1.43134 1.25295 1.15029 1.14356 1.1343 1.12589 1.13346 1.12925 1.13514 1.1343 1.12589 1.12673 1.12505 1.12336 1.12589 1.11916 1.12421 1.12252 1.12084 1.11579 1.11495 1.11916 1.12336 1.12252 1.11916 -5.80924e-08 -5.80924e-08 -5.82236e-08 -5.8617e-08 -5.8617e-08 -5.90978e-08 -5.9972e-08 -7.08999e-08 -7.3304e-08 -7.30417e-08 -7.31729e-08 -7.30417e-08 -7.30854e-08 -7.31729e-08 -7.29106e-08 -7.32603e-08 -7.32166e-08 -7.3304e-08 -7.3304e-08 -7.31292e-08 -7.35663e-08 -7.31729e-08 -7.31729e-08 -7.33914e-08 -7.361e-08 -7.361e-08 -7.3304e-08 -7.37411e-08 -7.38285e-08 -7.361e-08 -7.34351e-08 -7.38285e-08 -7.42656e-08 -7.38723e-08 -7.40034e-08 -7.39597e-08 -7.43968e-08 -7.43094e-08 -7.45279e-08 -7.45716e-08 -7.45716e-08 -7.48339e-08 -7.47465e-08 -7.49213e-08 -7.47902e-08 -7.4965e-08 -7.4659e-08 -7.5271e-08 -7.52273e-08 -7.54896e-08 -7.56207e-08 -7.5883e-08 -7.58393e-08 -7.57518e-08 -7.61015e-08 -7.64512e-08 -7.6189e-08 -7.64075e-08 -7.64512e-08 -7.68009e-08 -7.66261e-08 -7.68009e-08 -7.7238e-08 -7.72817e-08 -7.74129e-08 -7.76314e-08 -7.81123e-08 -7.81123e-08 -7.84182e-08 -7.82434e-08 -7.85057e-08 -7.89428e-08 -7.88554e-08 -7.87242e-08 -7.93362e-08 -7.94236e-08 -7.98607e-08 -7.98607e-08 -7.9817e-08 -8.02541e-08 -8.01667e-08 -8.04727e-08 -8.09535e-08 -8.13906e-08 -8.09535e-08 -8.15655e-08 -8.16966e-08 -8.19151e-08 -8.23523e-08 -8.27019e-08 -8.29205e-08 -8.25708e-08 -8.32702e-08 -8.32702e-08 -8.37947e-08 -8.41881e-08 -8.44941e-08 -8.46252e-08 -8.41881e-08 -8.38822e-08 -8.37073e-08 -8.38384e-08 -8.47564e-08 -8.58929e-08 -8.70294e-08 -8.71605e-08 -8.6942e-08 -8.77725e-08 -8.7991e-08 -8.8297e-08 -8.86467e-08 -8.90838e-08 -8.94772e-08 -9.00018e-08 -9.03077e-08 -9.06137e-08 -9.10508e-08 -9.13131e-08 -9.13131e-08 -9.19251e-08 -9.2843e-08 -9.30178e-08 -9.31053e-08 -9.36735e-08 -9.41543e-08 -9.47663e-08 -9.50286e-08 -9.58154e-08 -9.63836e-08 -9.65147e-08 -9.67333e-08 -9.67333e-08 -9.74327e-08 -9.80447e-08 -9.84381e-08 -9.89189e-08 -9.90937e-08 -9.95745e-08 -1.0058e-07 -1.0093e-07 -1.01367e-07 -1.01498e-07 -1.00667e-07 -9.96183e-08 -1.00055e-07 -1.00274e-07 -1.01104e-07 -1.01498e-07 -1.0246e-07 -1.02984e-07 -1.03771e-07 -1.04121e-07 -1.04252e-07 -1.04077e-07 -1.0364e-07 -1.03246e-07 -1.02634e-07 -1.02066e-07 -1.01542e-07 -1.0141e-07 -1.02197e-07 -1.08667e-07 -1.10153e-07 -1.09847e-07 -1.09453e-07 -1.09147e-07 -1.08754e-07 -1.08273e-07 -1.07967e-07 -1.07661e-07 -1.07399e-07 -1.07137e-07 -2.05235 -2.04646 -2.03888 -1.98166 -1.63077 -1.65854 -1.63161 -1.94043 -2.00775 -2.00186 -1.99765 -1.99092 -1.98587 -1.94885 -1.83609 -1.8748 -1.9682 -1.96147 -1.95979 -1.95474 -1.94969 -1.94801 -1.93959 -1.9194 -1.90425 -1.90004 -1.88994 -1.88321 -1.87564 -1.86807 -1.86218 -1.84787 -1.84282 -1.83357 -1.83861 -1.83188 -1.82767 -1.81926 -1.77298 -1.76372 -1.75867 -1.75867 -1.76372 -1.7713 -1.76625 -1.77887 -1.77298 -1.76793 -1.76036 -1.76204 -1.75615 -1.75447 -1.74605 -1.74269 -1.73932 -1.73427 -1.73427 -1.72922 -1.72417 -1.71828 -1.71828 -1.71071 -1.70987 -1.7065 -1.70061 -1.69977 -1.69977 -1.6922 -1.5887 -1.5441 -1.52559 -1.51128 -1.51633 -1.51128 -1.50455 -1.51212 -1.4995 -1.51044 -1.51465 -1.51296 -1.51128 -1.50792 -1.50792 -1.50876 -1.50623 -1.50539 -1.50623 -1.50371 -1.50539 -1.50371 -1.50876 -1.4995 -1.50455 -1.50707 -1.50539 -1.50287 -1.50623 -1.50455 -1.50707 -1.49866 -1.51128 -1.50203 -1.51381 -1.51212 -1.51717 -1.51044 -1.51044 -1.51296 -1.50876 -1.52306 -1.51465 -1.51549 -1.51633 -1.5197 -1.51717 -1.56514 -1.61478 -1.58197 -1.56682 -1.56682 -1.56514 -1.57187 -1.57523 -1.57776 -0.143892 -0.133794 -0.127904 -0.124538 -0.119489 -0.12033 -0.122013 -0.122855 -0.122013 -0.128745 -0.124538 -0.123696 -0.123696 -0.129587 -0.125379 -0.125379 -0.121172 -0.127062 -0.128745 -0.121172 -0.125379 -0.124538 -0.127904 -0.131269 -0.127062 -0.127904 -0.125379 -0.131269 -0.125379 -0.124538 -0.131269 -0.125379 -0.129587 -0.136318 -0.131269 -0.131269 -0.128745 -0.123696 -0.127904 -0.127062 -0.127062 -0.128745 -0.128745 -0.130428 -0.127062 -0.132952 -0.127904 -0.129587];
y=[-0.0611736 -0.0611736 -0.056596 -0.0603413 -0.056596 -0.0574283 -0.0611736 -0.0653351 -0.0628382 -0.0620059 -0.0607574 -0.0615897 -0.0557637 -0.0615897 -0.059509 -0.0611736 -0.0582606 -0.0578444 -0.059509 -0.0590929 -0.0586767 -0.059509 -0.0578444 -0.0603413 -0.0611736 -0.062422 -0.0599251 -0.0599251 -0.0611736 -0.059509 -0.0611736 -0.062422 -0.0603413 -0.0615897 -0.0603413 -0.062422 -0.0632543 -0.0628382 -0.0611736 -0.0615897 -0.0582606 -0.0586767 -0.0615897 -0.0711611 -0.123596 -0.156887 -0.171453 -0.179775 -0.885144 -0.882648 -0.882231 -0.880151 -0.880151 -0.87807 -0.875573 -0.874741 -0.879318 -0.876821 -0.876821 -0.875989 -0.870995 -0.873908 -0.874325 -0.87266 -0.870995 -0.873076 -0.867666 -0.870995 -0.870163 -0.868082 -0.868498 -0.865169 -0.868498 -0.866834 -0.866002 -0.868082 -0.866834 -0.868915 -0.866418 -0.864753 -0.862256 -0.866002 -0.864337 -0.865585 -0.866002 -0.866002 -0.865585 -0.865169 -0.868082 -0.869747 -0.868915 -0.869331 -0.871828 -0.868915 -0.86725 -0.868915 -0.871412 -0.870579 -0.868082 -0.870579 -0.873492 -0.874325 -0.872244 -0.874325 -0.874325 -0.875157 -0.875573 -0.87266 -0.878902 -0.879318 -0.88348 -0.883064 -0.879318 -0.881815 -0.880567 -0.885561 -0.880983 -0.886809 -0.885561 -0.887641 -0.885977 -0.889306 -0.891387 -0.891803 -0.892635 -0.891803 -0.892219 -0.899709 -0.899293 -0.89638 -0.897629 -0.913859 -0.925511 -0.930504 -0.932169 -0.924262 -0.910529 -0.9072 -0.903455 -0.890554 -0.852685 -0.842281 -0.837287 -0.83812 -0.841033 -0.842281 -0.844362 -0.846027 -0.848523 -0.850604 -0.854766 -0.856014 -0.857262 -0.859759 -0.86184 -0.863921 -0.864753 -0.869331 -0.870163 -0.87266 -0.875573 -0.878902 -0.880983 -0.884728 -0.887225 -0.893051 -0.953392 -1.00999 -1.01332 -1.01373 -1.01748 -1.02122 -1.02497 1.33617 1.34319 1.34535 1.34805 1.35454 1.35724 1.36372 1.36859 1.37345 1.38101 1.38533 1.39182 1.39668 1.40316 1.40533 1.40857 1.41181 1.39884 1.3956 1.40587 1.41019 1.41775 1.42262 1.42694 1.43126 1.4372 1.4345 1.42586 1.41505 1.40533 1.39506 1.38371 1.37615 1.36696 1.354 1.34697 1.33779 1.32914 1.32158 1.31239 1.30645 1.29673 1.28592 1.27998 1.27295 1.26431 1.25458 1.24972 1.2427 1.23405 1.23027 1.22433 1.21946 1.21082 1.20433 1.19731 1.19191 1.19083 1.17624 1.17732 1.16597 1.16543 1.15517 1.14976 1.14706 1.14274 1.13463 1.13085 1.12815 1.12005 1.1168 1.11032 1.10762 1.1033 1.09789 1.09303 1.09087 1.08493 1.08547 1.07682 1.0725 1.06818 1.06007 1.05791 1.05359 1.05035 1.05143 1.04603 1.0417 1.03954 1.039 1.02874 1.0309 1.02387 1.02279 1.01847 1.01577 1.01199 1.00982 1.01091 1.0082 1.00172 0.997398 0.995237 0.993616 0.988213 0.990374 0.986052 0.98389 0.98389 0.976867 0.974705 0.974165 0.971464 0.968762 0.972004 0.96552 0.962278 0.962278 0.957416 0.959037 0.957416 0.955254 0.948231 0.94715 0.94661 0.948231 0.943368 0.942287 0.941207 0.938505 0.937424 0.934183 0.934183 0.933642 0.930401 0.92932 0.930941 0.926618 0.924457 0.926618 0.926618 0.926078 0.922836 0.917433 0.917974 0.915812 0.916893 0.807212 0.741835 0.734271 0.727247 0.720763 0.723465 0.726707 0.721844 0.726707 0.727787 0.726166 0.724545 0.721844 0.721304 0.721304 0.721304 0.722924 0.720223 0.719683 0.716981 0.718602 0.720763 0.715901 0.722924 1.322 1.328 1.331 1.341 1.337 1.345 1.369 1.622 1.67 1.669 1.667 1.667 1.674 1.674 1.676 1.67 1.675 1.68 1.679 1.679 1.673 1.677 1.673 1.677 1.681 1.684 1.689 1.681 1.682 1.68 1.681 1.693 1.69 1.689 1.696 1.694 1.7 1.695 1.707 1.707 1.705 1.7 1.713 1.718 1.713 1.713 1.721 1.723 1.723 1.723 1.731 1.731 1.735 1.74 1.745 1.74 1.752 1.745 1.748 1.763 1.766 1.762 1.762 1.769 1.774 1.776 1.781 1.782 1.791 1.789 1.793 1.796 1.803 1.805 1.807 1.812 1.816 1.826 1.832 1.843 1.838 1.847 1.849 1.851 1.862 1.865 1.872 1.876 1.882 1.881 1.9 1.9 1.908 1.908 1.915 1.925 1.941 1.931 1.925 1.932 1.92 1.917 1.927 1.969 1.987 1.996 1.992 2.007 2.013 2.021 2.028 2.036 2.048 2.059 2.061 2.075 2.078 2.092 2.108 2.112 2.114 2.128 2.133 2.146 2.162 2.171 2.176 2.191 2.203 2.209 2.21 2.215 2.23 2.245 2.252 2.261 2.271 2.283 2.294 2.306 2.315 2.322 2.301 2.278 2.279 2.294 2.312 2.325 2.339 2.357 2.372 2.383 2.393 2.381 2.371 2.36 2.348 2.337 2.324 2.313 2.339 2.483 2.518 2.513 2.501 2.496 2.485 2.477 2.469 2.462 2.453 2.445 1.31942 1.31564 1.31023 1.27349 1.0444 1.06386 1.04765 1.24486 1.28754 1.28646 1.2816 1.2789 1.27673 1.25242 1.1784 1.20704 1.26161 1.2589 1.25836 1.2535 1.25296 1.25026 1.24702 1.23189 1.22216 1.21946 1.21406 1.2092 1.20541 1.20217 1.19461 1.18542 1.18434 1.1784 1.17678 1.17786 1.17084 1.16705 1.13788 1.13247 1.12923 1.12815 1.13247 1.13734 1.13518 1.14328 1.1368 1.13301 1.13247 1.12815 1.12761 1.12383 1.12059 1.1168 1.11789 1.1141 1.11086 1.1087 1.106 1.10222 1.10222 1.1006 1.09681 1.09573 1.09141 1.09249 1.09195 1.08655 1.01847 0.991995 0.977407 0.971464 0.972004 0.968222 0.96552 0.963899 0.969843 0.968762 0.969302 0.973625 0.973085 0.968762 0.970923 0.970383 0.966601 0.966061 0.968762 0.963899 0.967682 0.968222 0.96444 0.967141 0.966601 0.967141 0.962819 0.963899 0.967682 0.962279 0.969302 0.968222 0.967682 0.970383 0.966601 0.967682 0.969843 0.969843 0.967141 0.971464 0.973085 0.974165 0.973625 0.974705 0.970383 0.970923 0.975246 1.00496 1.04224 1.01901 1.00496 1.00496 1.00604 1.00658 1.00983 1.01091 0.0896902 0.0864484 0.0826663 0.0799647 0.0794244 0.0832066 0.0778035 0.0794244 0.0805051 0.0794244 0.0794244 0.0810454 0.0805051 0.0799647 0.0799647 0.0794244 0.0778035 0.0794244 0.0815857 0.0815857 0.082126 0.0805051 0.0805051 0.0805051 0.0832066 0.0815857 0.0848275 0.0832066 0.0832066 0.0799647 0.082126 0.0788841 0.0826663 0.0826663 0.0810454 0.0810454 0.0788841 0.0815857 0.082126 0.0805051 0.0799647 0.0783438 0.0810454 0.0837469 0.0853678 0.0815857 0.082126 0.0853678];
scatter(x,y)
hold on
axis([-5 5 -5 5])