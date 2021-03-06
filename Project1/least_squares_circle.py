#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
http://www.scipy.org/Cookbook/Least_Squares_Circle
"""

from numpy import *

# Coordinates of the 2D points

x = r_[  9, 35, -13,  10,  23,   0]
y = r_[ 34, 10,   6, -14,  27, -10]
basename = 'circle'

# x = r_[36, 36, 19, 18, 33, 26]
# y = r_[14, 10, 28, 31, 18, 26]
# basename = 'arc'

# # Code to generate random data points
R0 = 25
nb_pts = 40
dR = 1
angle =10*pi/5
theta0 = random.uniform(0, angle, size=nb_pts)
# x = (10 + R0*cos(theta0) + dR*random.normal(size=nb_pts)).round()
# y = (10 + R0*sin(theta0) + dR*random.normal(size=nb_pts)).round()
#
# print x
# print y

# approximation for this one is not very good
# x = r_[2.0441416844543063, -4.8473419604357595, 6.035924367609888, 0.61305246847457, 3.1060379578679838, 3.12729392855763, 9.21433857702771, 4.840608948458536, 5.810352396544397, 3.501243141339829, 2.0408479885250133, 3.9553148320805063, 2.937155805658797, 1.985461121758049, 2.5314507109540187, -9.637014168746475, -4.29530717714276, -6.852004979443002, -3.0628719465151684, -9.038982352729231, -6.209383645113896, -5.986458867425609, -6.503231539187842, -8.8206410449103, -8.84838371131579, -3.402478821067603, -7.182216926525696, -6.756070990712349, -6.991222401520425, -8.436962465580622, -0.6066808708053874, 2.8953991559600007, -2.3834962765307837, 7.452468713330919, 2.2372293049484995, 7.6567265449769835, 8.182768983916265, 4.372254918831211, 8.631071420871237, 7.715201867994537, 7.059444306377918, 3.0833236002680025, 4.644183613399987, 3.935423541395201, 2.412013558611222, -1.4217812360224624, -0.9832208008049845, -4.705031609278221, -5.359189140259455, -2.2465475134221915, -8.357495252436035, -6.851374200710136, -9.683855481741222, -13.234993453320978, -9.687009177470664, -6.308869879285872, -4.326286295085511, -4.294525547288134, -3.9578347127464943, 3.748037439904194]
# y = r_[9.58896174150486, 9.476888258879157, 14.07841638928091, 13.384871312813294, 16.230042584476124, 15.087581486113733, 15.837697555586592, 17.367334846432257, 15.694258581512727, 26.839754928371114, 20.228055872317775, 24.366570606056793, 24.621882414679373, 16.53688629297111, 23.712523236632055, 22.943096056017062, 21.48141581798043, 22.22328443041047, 21.48583971573433, 21.043111014501132, 21.231373873750005, 19.53518584249204, 17.085532207228017, 13.80125755976924, 10.86945005780052, 12.569319454779915, 10.27837711535022, 8.956134054004941, 10.315630597511046, 13.549822940262807, 6.143462326200005, 13.975242483836848, 6.886206594399699, 13.487077064100482, 10.66564046921024, 14.648446539668152, 18.323708192706857, 12.639033877456606, 18.881888582246876, 15.535809835246319, 20.26804752798966, 23.584607791569503, 23.15671241748706, 22.747498154911003, 18.91803106779434, 25.05960350747684, 23.51308665139784, 25.87191226635187, 23.72983434276249, 16.1264530293381, 21.29093507837618, 16.569606777492215, 17.408041200064012, 17.97434662066741, 18.290595746259886, 19.275527961102394, 10.822607828871924, 11.106761823572779, 9.088382433612807, 11.048489764613901]
# x_actual = r_[0.0, 1.4672214011007085, 2.83753958756461, 4.0510650791270315, 5.054760988665319, 5.80476098866532, 6.268286480227742, 6.4250791751292216, 6.268286480227741, 5.804760988665318, 5.054760988665317, 4.051065079127029, 2.837539587564608, 1.4672214011007072, -8.881784197001252e-16, -1.5000000000000009, -2.9672214011007103, -4.337539587564613, -5.551065079127038, -6.554760988665329, -7.304760988665335, -7.768286480227762, -7.925079175129248, -7.768286480227774, -7.3047609886653575, -6.55476098866536, -5.551065079127074, -4.3375395875646525, -2.9672214011007503, -1.500000000000041, -4.107825191113079e-14, 1.4672214011006668, 2.837539587564567, 4.051065079126986, 5.054760988665272, 5.8047609886652705, 6.268286480227689, 6.4250791751291665, 6.268286480227683, 5.804760988665258, 5.0547609886652545, 4.051065079126965, 2.8375395875645424, 1.4672214011006404, -6.816769371198461e-14, -1.5000000000000682, -2.967221401100777, -4.337539587564679, -5.551065079127102, -6.554760988665391, -7.304760988665395, -7.76828648022782, -7.925079175129303, -7.768286480227826, -7.304760988665407, -6.55476098866541, -5.551065079127124, -4.337539587564702, -2.9672214011008, -1.5000000000000908]
# y_actual = r_[10.0, 10.31186753622664, 10.92197250084034, 11.803650379279048, 12.91836761749514, 14.217405723171797, 15.643990497614528, 17.135773340666937, 18.627556183719346, 20.054140958162076, 21.353179063838734, 22.467896302054825, 23.349574180493534, 23.959679145107234, 24.271546681333874, 24.271546681333877, 23.95967914510724, 23.349574180493544, 22.46789630205484, 21.353179063838752, 20.054140958162098, 18.62755618371937, 17.13577334066696, 15.643990497614551, 14.217405723171819, 12.918367617495159, 11.803650379279066, 10.921972500840356, 10.311867536226657, 10.000000000000021, 10.000000000000025, 10.311867536226666, 10.921972500840369, 11.80365037927908, 12.918367617495173, 14.217405723171833, 15.643990497614563, 17.135773340666972, 18.62755618371938, 20.05414095816211, 21.353179063838766, 22.467896302054854, 23.349574180493562, 23.95967914510726, 24.2715466813339, 24.2715466813339, 23.95967914510726, 23.349574180493562, 22.467896302054854, 21.353179063838766, 20.05414095816211, 18.62755618371938, 17.135773340666972, 15.643990497614562, 14.217405723171831, 12.918367617495171, 11.803650379279079, 10.921972500840369, 10.31186753622667, 10.000000000000034]

# with two circles
# x = r_ [1.7394861622815916, 3.3848196429534525, 5.330496957155588, 3.2282651263943225, 7.0235765114902105, 4.737218252804991, -0.5819413897099581, 13.560312985926558, 3.9245222816245082, 3.240327981470707, 3.2978129369788167, 1.4045845311593337, 1.792290138053336, 1.2095893681761358, 2.252874453942855, 2.833664162319776, 0.6713150765065334, -4.14325933280239, -5.820699803375332, -8.909994041386192, -9.225855260141559, -8.930377933105413, -9.801897155023788, -6.845409884981802, -9.871117359779795, -8.127553663318057, -6.474551698982755, -3.1042451748513926, -3.1436294992111855, -3.583285662039512, 2.321892066439589, 4.596106629861195, 3.148230351252571, 0.7835146904284178, 10.399408615502502, 8.224903695773467, 7.4476962925997965, 6.565628787591956, 11.290334851792274, 6.542390029414056, 1.4715351664550274, 6.43237598551133, 6.681989536963721, 2.8783194753434653, -2.7040600487155837, 2.22928199506417, -4.619695073816294, -7.1065508587331925, -2.470941178995336, -5.497099765265109, -5.193093272385381, -7.358987078856403, -8.249278110381105, -8.47635160461728, -9.455556397602738, -3.6052492109670498, -7.184360359902821, -7.594491111732696, -3.2207927920369737, 0.9353548885349576]
# y = r_ [10.513409535174153, 9.840603020212116, 11.121885423733275, 18.447318957026347, 11.300602749240808, 15.861316047013677, 15.1176597237682, 20.910635149255086, 20.718923187279902, 17.962300456210386, 18.169714495273546, 25.572323242351217, 18.04972866367403, 24.09892831860164, 31.33044119920509, 24.02060256642733, 21.233767634683375, 22.810622505449036, 22.890668471495896, 21.888248140616188, 22.872993387650908, 19.060839583552024, 22.300156442438322, 14.565904525094528, 12.205272876172126, 11.280358638521573, 16.76380846853116, 11.717904420907322, 6.7697710953561065, 11.168617849205827, 9.01690885673681, 17.519367010698005, 15.309447025449511, 12.291559864726615, 10.448205118382644, 17.91165150262593, 16.376817935341247, 13.103608361584914, 16.979711353159683, 14.411717690426919, 18.413635916383775, 22.800665714725383, 31.159125856448423, 24.71824690035979, 26.255737101609974, 30.431277075063953, 24.318839447934835, 23.45647474296725, 23.47312340214255, 28.253705496916197, 24.219287617640013, 18.731315176858484, 20.54711331403899, 15.348388025808145, 15.75457996752718, 13.05899422600076, 8.815648105056459, 8.685030360411398, 8.333077891549054, 13.062511149783345]
# x_actual = r_[0.0, 1.4672214011007085, 2.83753958756461, 4.0510650791270315, 5.054760988665319, 5.80476098866532, 6.268286480227742, 6.4250791751292216, 6.268286480227741, 5.804760988665318, 5.054760988665317, 4.051065079127029, 2.837539587564608, 1.4672214011007072, -8.881784197001252e-16, -1.5000000000000009, -2.9672214011007103, -4.337539587564613, -5.551065079127038, -6.554760988665329, -7.304760988665335, -7.768286480227762, -7.925079175129248, -7.768286480227774, -7.3047609886653575, -6.55476098866536, -5.551065079127074, -4.3375395875646525, -2.9672214011007503, -1.500000000000041, -4.107825191113079e-14, 1.4672214011006668, 2.837539587564567, 4.051065079126986, 5.054760988665272, 5.8047609886652705, 6.268286480227689, 6.4250791751291665, 6.268286480227683, 5.804760988665258, 5.0547609886652545, 4.051065079126965, 2.8375395875645424, 1.4672214011006404, -6.816769371198461e-14, -1.5000000000000682, -2.967221401100777, -4.337539587564679, -5.551065079127102, -6.554760988665391, -7.304760988665395, -7.76828648022782, -7.925079175129303, -7.768286480227826, -7.304760988665407, -6.55476098866541, -5.551065079127124, -4.337539587564702, -2.9672214011008, -1.5000000000000908]
# y_actual = r_ [10.0, 10.31186753622664, 10.92197250084034, 11.803650379279048, 12.91836761749514, 14.217405723171797, 15.643990497614528, 17.135773340666937, 18.627556183719346, 20.054140958162076, 21.353179063838734, 22.467896302054825, 23.349574180493534, 23.959679145107234, 24.271546681333874, 24.271546681333877, 23.95967914510724, 23.349574180493544, 22.46789630205484, 21.353179063838752, 20.054140958162098, 18.62755618371937, 17.13577334066696, 15.643990497614551, 14.217405723171819, 12.918367617495159, 11.803650379279066, 10.921972500840356, 10.311867536226657, 10.000000000000021, 10.000000000000025, 10.311867536226666, 10.921972500840369, 11.80365037927908, 12.918367617495173, 14.217405723171833, 15.643990497614563, 17.135773340666972, 18.62755618371938, 20.05414095816211, 21.353179063838766, 22.467896302054854, 23.349574180493562, 23.95967914510726, 24.2715466813339, 24.2715466813339, 23.95967914510726, 23.349574180493562, 22.467896302054854, 21.353179063838766, 20.05414095816211, 18.62755618371938, 17.135773340666972, 15.643990497614562, 14.217405723171831, 12.918367617495171, 11.803650379279079, 10.921972500840369, 10.31186753622667, 10.000000000000034]

# with three circles; the algebraic method appears more accurate
x = r_ [1.3905235974633778, 7.832049402391674, 9.020314462351989, 3.687485010010496, 4.933087914504979, 5.842278721831094, 7.374384722598193, 11.548435059429593, 10.08330407331331, 4.048332216167037, 11.753400231528877, 5.24814930818585, 0.3780861370621249, 3.819226471192638, 3.219009627591963, -0.660991670650813, -6.810902143188039, -2.648632039619782, -0.8651469995661083, -5.882826870804094, -6.771881598941817, -6.883212384180803, -10.970288387596014, -9.38416048883138, -2.0041408634734, -4.1999609273146294, -4.427392485133488, -4.094107923812431, -4.539507495972897, -6.958816977357863, -0.5837156626237164, 3.211904185873981, 2.6999090897786973, 4.946462347786874, 6.250544373196611, 1.4360007457367132, 6.376447944684558, 5.395872942944427, 3.6592834591198713, 3.6799054753035985, 6.372070482657179, 10.83971503990248, 6.471684767984395, -4.13122094642976, 1.3657905443071188, -2.1545837703277186, -3.6218498314375362, -5.990316163817381, -2.825131401522447, -8.47962049553197, -3.0710249464300077, -5.242374500395425, -11.356227320748491, -3.580215175474069, -8.949524398862625, -12.148475712671257, -3.541357682569754, -2.3218551317498113, -4.10750098248105, -1.154362522019348, 1.719708807469195, -2.9317706605909493, 5.581580285882415, 7.322885401113478, 8.035547065332059, 7.115626723235949, 6.8834109751060195, 9.605681429819468, 5.378480946301948, 4.55936497171603, 0.9696632799355314, 3.0281397205843197, -2.4315041509242974, -0.4213417059299991, 0.13371487118430309, -2.5616840637149783, -3.330922819747191, -3.238792470549532, -12.848697164298123, -11.650132469341331, -2.9250345334239203, -8.844969766009035, -7.54940178448948, -10.204223020208033, -7.4851192397126525, -9.156624882388012, -6.366425519643718, -8.553734125327592, -4.609302566545153, -4.264881667574036, 1.4533519725675805]
y = r_  [5.1592964225372135, 8.216934337621904, 14.221677454416298, 9.606966741201592, 15.393598119630198, 15.463991661062682, 18.798461424361072, 17.83425651827887, 16.480645145332982, 22.97048479933387, 19.510051232214657, 24.642699320629887, 25.45829257417096, 25.497397786842278, 22.450305133766594, 21.947846746481055, 18.037770599424036, 23.858011123610375, 19.308656588152285, 16.69873918296076, 20.488408553230123, 16.367880867164754, 23.654044378766407, 20.73648776720881, 15.90450795825495, 16.135338583542634, 8.493714748632618, 8.913938224865586, 8.734285273935438, 8.050993570472942, 10.158164230122795, 8.08449276240243, 14.886302280598855, 14.237310205824675, 17.306486687897937, 13.8601024213346, 17.90588766410904, 15.869047349619871, 20.686353443244506, 18.42398071352659, 23.22554467544738, 21.198830063607687, 24.727001975565397, 23.188652263477202, 25.512192853204706, 25.07747201310473, 24.054712728944807, 22.68899006890544, 22.641944066718526, 18.33228864387304, 17.94999176153331, 22.10590333682616, 19.005801408713417, 13.582191596376182, 10.639031293990215, 15.577633926934181, 9.911438955202737, 8.666040822401431, 11.027340108357107, 12.56573800372876, 11.771468252890216, 12.46635569231553, 8.986038622183006, 12.215863938535502, 10.291879983793017, 16.505270167068254, 19.6009636347262, 19.10972604819267, 15.892910888479538, 26.63984263799601, 17.951774058138326, 21.107945349676598, 23.441244468726836, 21.853959925952115, 21.162499359087953, 21.808181391384416, 20.52011983367284, 19.153098462721978, 26.471740153772178, 25.208554508505728, 17.67356964440606, 18.51548339755294, 16.646498463883713, 15.599794378503121, 17.747778170685166, 13.998749493955843, 13.588001144109704, 16.94352162086974, 9.476077823803571, 5.4278110057897955, 12.342893333479008]
x_actual = r_ [0.0, 1.4672214011007085, 2.83753958756461, 4.0510650791270315, 5.054760988665319, 5.80476098866532, 6.268286480227742, 6.4250791751292216, 6.268286480227741, 5.804760988665318, 5.054760988665317, 4.051065079127029, 2.837539587564608, 1.4672214011007072, -8.881784197001252e-16, -1.5000000000000009, -2.9672214011007103, -4.337539587564613, -5.551065079127038, -6.554760988665329, -7.304760988665335, -7.768286480227762, -7.925079175129248, -7.768286480227774, -7.3047609886653575, -6.55476098866536, -5.551065079127074, -4.3375395875646525, -2.9672214011007503, -1.500000000000041, -4.107825191113079e-14, 1.4672214011006668, 2.837539587564567, 4.051065079126986, 5.054760988665272, 5.8047609886652705, 6.268286480227689, 6.4250791751291665, 6.268286480227683, 5.804760988665258, 5.0547609886652545, 4.051065079126965, 2.8375395875645424, 1.4672214011006404, -6.816769371198461e-14, -1.5000000000000682, -2.967221401100777, -4.337539587564679, -5.551065079127102, -6.554760988665391, -7.304760988665395, -7.76828648022782, -7.925079175129303, -7.768286480227826, -7.304760988665407, -6.55476098866541, -5.551065079127124, -4.337539587564702, -2.9672214011008, -1.5000000000000908, -9.08162434143378e-14, 1.467221401100617, 2.837539587564517, 4.0510650791269365, 5.0547609886652225, 5.804760988665221, 6.26828648022764, 6.425079175129117, 6.2682864802276335, 5.804760988665208, 5.054760988665205, 4.051065079126915, 2.8375395875644926, 1.4672214011005906, -1.1790568521519162e-13, -1.500000000000118, -2.9672214011008267, -4.337539587564729, -5.551065079127151, -6.554760988665441, -7.3047609886654445, -7.76828648022787, -7.925079175129353, -7.768286480227876, -7.304760988665457, -6.55476098866546, -5.551065079127174, -4.337539587564752, -2.9672214011008498, -1.5000000000001406, -1.4055423491754482e-13]
y_actual = r_ [10.0, 10.31186753622664, 10.92197250084034, 11.803650379279048, 12.91836761749514, 14.217405723171797, 15.643990497614528, 17.135773340666937, 18.627556183719346, 20.054140958162076, 21.353179063838734, 22.467896302054825, 23.349574180493534, 23.959679145107234, 24.271546681333874, 24.271546681333877, 23.95967914510724, 23.349574180493544, 22.46789630205484, 21.353179063838752, 20.054140958162098, 18.62755618371937, 17.13577334066696, 15.643990497614551, 14.217405723171819, 12.918367617495159, 11.803650379279066, 10.921972500840356, 10.311867536226657, 10.000000000000021, 10.000000000000025, 10.311867536226666, 10.921972500840369, 11.80365037927908, 12.918367617495173, 14.217405723171833, 15.643990497614563, 17.135773340666972, 18.62755618371938, 20.05414095816211, 21.353179063838766, 22.467896302054854, 23.349574180493562, 23.95967914510726, 24.2715466813339, 24.2715466813339, 23.95967914510726, 23.349574180493562, 22.467896302054854, 21.353179063838766, 20.05414095816211, 18.62755618371938, 17.135773340666972, 15.643990497614562, 14.217405723171831, 12.918367617495171, 11.803650379279079, 10.921972500840369, 10.31186753622667, 10.000000000000034, 10.000000000000037, 10.311867536226679, 10.921972500840381, 11.803650379279093, 12.918367617495186, 14.217405723171845, 15.643990497614576, 17.135773340666987, 18.627556183719395, 20.054140958162126, 21.35317906383878, 22.467896302054868, 23.349574180493576, 23.959679145107273, 24.271546681333913, 24.271546681333913, 23.959679145107273, 23.349574180493576, 22.467896302054868, 21.35317906383878, 20.054140958162126, 18.627556183719395, 17.135773340666987, 15.643990497614576, 14.217405723171845, 12.918367617495186, 11.803650379279093, 10.921972500840383, 10.311867536226684, 10.000000000000048, 10.000000000000052]




# == METHOD 1 ==
method_1 = 'algebraic'

# coordinates of the barycenter
x_m = mean(x)
y_m = mean(y)

# calculation of the reduced coordinates
u = x - x_m
v = y - y_m

# linear system defining the center in reduced coordinates (uc, vc):
#    Suu * uc +  Suv * vc = (Suuu + Suvv)/2
#    Suv * uc +  Svv * vc = (Suuv + Svvv)/2
Suv  = sum(u*v)
Suu  = sum(u**2)
Svv  = sum(v**2)
Suuv = sum(u**2 * v)
Suvv = sum(u * v**2)
Suuu = sum(u**3)
Svvv = sum(v**3)

# Solving the linear system
A = array([ [ Suu, Suv ], [Suv, Svv]])
B = array([ Suuu + Suvv, Svvv + Suuv ])/2.0
uc, vc = linalg.solve(A, B)

xc_1 = x_m + uc
yc_1 = y_m + vc

# Calculation of all distances from the center (xc_1, yc_1)
Ri_1      = sqrt((x-xc_1)**2 + (y-yc_1)**2)
R_1       = mean(Ri_1)
residu_1  = sum((Ri_1-R_1)**2)
residu2_1 = sum((Ri_1**2-R_1**2)**2)

# Decorator to count functions calls
import functools
def countcalls(fn):
    "decorator function count function calls "

    @functools.wraps(fn)
    def wrapped(*args):
        wrapped.ncalls +=1
        return fn(*args)

    wrapped.ncalls = 0
    return wrapped

#  == METHOD 2 ==
# Basic usage of optimize.leastsq
from scipy      import optimize


# ------------------------------------------------------------------------
method_2  = "leastsq"

def calc_R(xc, yc):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    return sqrt((x-xc)**2 + (y-yc)**2)

@countcalls
def f_2(c):
    """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c)
    return Ri - Ri.mean()

center_estimate = x_m, y_m
center_2, ier = optimize.leastsq(f_2, center_estimate)

xc_2, yc_2 = center_2
Ri_2       = calc_R(xc_2, yc_2)
R_2        = Ri_2.mean()
residu_2   = sum((Ri_2 - R_2)**2)
residu2_2  = sum((Ri_2**2-R_2**2)**2)
ncalls_2   = f_2.ncalls

# == METHOD 2b ==
# Advanced usage, with jacobian
method_2b  = "leastsq with jacobian"

def calc_R(xc, yc):
    """ calculate the distance of each 2D points from the center c=(xc, yc) """
    return sqrt((x-xc)**2 + (y-yc)**2)

@countcalls
def f_2b(c):
    """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c)
    return Ri - Ri.mean()

@countcalls
def Df_2b(c):
    """ Jacobian of f_2b
    The axis corresponding to derivatives must be coherent with the col_deriv option of leastsq"""
    xc, yc     = c
    df2b_dc    = empty((len(c), x.size))

    Ri = calc_R(xc, yc)
    df2b_dc[ 0] = (xc - x)/Ri                   # dR/dxc
    df2b_dc[ 1] = (yc - y)/Ri                   # dR/dyc
    df2b_dc       = df2b_dc - df2b_dc.mean(axis=1)[:, newaxis]

    return df2b_dc

center_estimate = x_m, y_m
center_2b, ier = optimize.leastsq(f_2b, center_estimate, Dfun=Df_2b, col_deriv=True)

xc_2b, yc_2b = center_2b
Ri_2b        = calc_R(xc_2b, yc_2b)
R_2b         = Ri_2b.mean()
residu_2b    = sum((Ri_2b - R_2b)**2)
residu2_2b   = sum((Ri_2b**2-R_2b**2)**2)
ncalls_2b    = f_2b.ncalls

print """
Method 2b :
print "Functions calls : f_2b=%d Df_2b=%d""" % ( f_2b.ncalls, Df_2b.ncalls)

# == METHOD 3 ==
# Basic usage of odr with an implicit function definition
from scipy      import  odr

method_3  = "odr"

@countcalls
def f_3(beta, x):
    """ implicit definition of the circle """
    return (x[0]-beta[0])**2 + (x[1]-beta[1])**2 -beta[2]**2

# initial guess for parameters
R_m = calc_R(x_m, y_m).mean()
beta0 = [ x_m, y_m, R_m]

# for implicit function :
#       data.x contains both coordinates of the points
#       data.y is the dimensionality of the response
lsc_data   = odr.Data(row_stack([x, y]), y=1)
lsc_model  = odr.Model(f_3, implicit=True)
lsc_odr    = odr.ODR(lsc_data, lsc_model, beta0)
lsc_out    = lsc_odr.run()

xc_3, yc_3, R_3 = lsc_out.beta
Ri_3       = calc_R(xc_3, yc_3)
residu_3   = sum((Ri_3 - R_3)**2)
residu2_3  = sum((Ri_3**2-R_3**2)**2)
ncalls_3   = f_3.ncalls

# == METHOD 3b ==
# Advanced usage, with jacobian
method_3b  = "odr with jacobian"
print "\nMethod 3b : ", method_3b


@countcalls
def f_3b(beta, x):
    """ implicit definition of the circle """
    return (x[0]-beta[0])**2 + (x[1]-beta[1])**2 -beta[2]**2


@countcalls
def jacb(beta, x):
    """ Jacobian function with respect to the parameters beta.
    return df_3b/dbeta
    """
    xc, yc, r = beta
    xi, yi    = x

    df_db    = empty((beta.size, x.shape[1]))
    df_db[0] =  2*(xc-xi)                     # d_f/dxc
    df_db[1] =  2*(yc-yi)                     # d_f/dyc
    df_db[2] = -2*r                           # d_f/dr

    return df_db


@countcalls
def jacd(beta, x):
    """ Jacobian function with respect to the input x.
    return df_3b/dx
    """
    xc, yc, r = beta
    xi, yi    = x

    df_dx    = empty_like(x)
    df_dx[0] =  2*(xi-xc)                     # d_f/dxi
    df_dx[1] =  2*(yi-yc)                     # d_f/dyi

    return df_dx


def calc_estimate(data):
    """ Return a first estimation on the parameter from the data  """
    xc0, yc0 = data.x.mean(axis=1)
    r0 = sqrt((data.x[0]-xc0)**2 +(data.x[1] -yc0)**2).mean()
    return xc0, yc0, r0



# for implicit function :
#       data.x contains both coordinates of the points
#       data.y is the dimensionality of the response
lsc_data  = odr.Data(row_stack([x, y]), y=1)
lsc_model = odr.Model(f_3b, implicit=True, estimate=calc_estimate, fjacd=jacd, fjacb=jacb)
lsc_odr   = odr.ODR(lsc_data, lsc_model)    # beta0 has been replaced by an estimate function
lsc_odr.set_job(deriv=3)                    # use user derivatives function without checking
lsc_odr.set_iprint(iter=1, iter_step=1)     # print details for each iteration
lsc_out   = lsc_odr.run()

xc_3b, yc_3b, R_3b = lsc_out.beta
Ri_3b       = calc_R(xc_3b, yc_3b)
residu_3b   = sum((Ri_3b - R_3b)**2)
residu2_3b  = sum((Ri_3b**2-R_3b**2)**2)
ncalls_3b   = f_3b.ncalls

print "\nFunctions calls : f_3b=%d jacb=%d jacd=%d" % (f_3b.ncalls, jacb.ncalls, jacd.ncalls)


# Summary
fmt = '%-22s %10.5f %10.5f %10.5f %10d %10.6f %10.6f %10.2f'
print ('\n%-22s' +' %10s'*7) % tuple('METHOD Xc Yc Rc nb_calls std(Ri) residu residu2'.split())
print '-'*(22 +7*(10+1))
print  fmt % (method_1 , xc_1 , yc_1 , R_1 ,        1 , Ri_1.std() , residu_1 , residu2_1 )
print  fmt % (method_2 , xc_2 , yc_2 , R_2 , ncalls_2 , Ri_2.std() , residu_2 , residu2_2 )
print  fmt % (method_2b, xc_2b, yc_2b, R_2b, ncalls_2b, Ri_2b.std(), residu_2b, residu2_2b)
print  fmt % (method_3 , xc_3 , yc_3 , R_3 , ncalls_3 , Ri_3.std() , residu_3 , residu2_3 )
print  fmt % (method_3b, xc_3b, yc_3b, R_3b, ncalls_3b, Ri_3b.std(), residu_3b, residu2_3b)

# plotting functions
from matplotlib                 import pyplot as p, cm, colors
p.close('all')

def plot_all(residu2=False):
    """ Draw data points, best fit circles and center for the three methods,
    and adds the iso contours corresponding to the fiel residu or residu2
    """

    f = p.figure( facecolor='white')  #figsize=(7, 5.4), dpi=72,
    p.axis('equal')

    theta_fit = linspace(-pi, pi, 180)

    x_fit1 = xc_1 + R_1*cos(theta_fit)
    y_fit1 = yc_1 + R_1*sin(theta_fit)
    p.plot(x_fit1, y_fit1, 'b-' , label=method_1, lw=2)

    x_fit2 = xc_2 + R_2*cos(theta_fit)
    y_fit2 = yc_2 + R_2*sin(theta_fit)
    p.plot(x_fit2, y_fit2, 'k--', label=method_2, lw=2)

    x_fit3 = xc_3 + R_3*cos(theta_fit)
    y_fit3 = yc_3 + R_3*sin(theta_fit)
    p.plot(x_fit3, y_fit3, 'r-.', label=method_3, lw=2)

    p.plot([xc_1], [yc_1], 'bD', mec='y', mew=1)
    p.plot([xc_2], [yc_2], 'gD', mec='r', mew=1)
    p.plot([xc_3], [yc_3], 'kD', mec='w', mew=1)

    # draw
    p.xlabel('x')
    p.ylabel('y')

    # plot the residu fields
    nb_pts = 100

    p.draw()
    xmin, xmax = p.xlim()
    ymin, ymax = p.ylim()

    vmin = min(xmin, ymin)
    vmax = max(xmax, ymax)

    xg, yg = ogrid[vmin:vmax:nb_pts*1j, vmin:vmax:nb_pts*1j]
    xg = xg[..., newaxis]
    yg = yg[..., newaxis]

    Rig    = sqrt( (xg - x)**2 + (yg - y)**2 )
    Rig_m  = Rig.mean(axis=2)[..., newaxis]

    if residu2 : residu = sum( (Rig**2 - Rig_m**2)**2 ,axis=2)
    else       : residu = sum( (Rig-Rig_m)**2 ,axis=2)

    lvl = exp(linspace(log(residu.min()), log(residu.max()), 15))

    p.contourf(xg.flat, yg.flat, residu.T, lvl, alpha=0.4, cmap=cm.Purples_r) # , norm=colors.LogNorm())
    cbar = p.colorbar(fraction=0.175, format='%.f')
    p.contour (xg.flat, yg.flat, residu.T, lvl, alpha=0.8, colors="lightblue")

    if residu2 : cbar.set_label('Residu_2 - algebraic approximation')
    else       : cbar.set_label('Residu')

    # plot data
    p.plot(x, y, 'ro', label='data', ms=8, mec='b', mew=1)
    p.plot(x_actual, y_actual, color='black', label='actual data', ms=8, mec='b', mew=1)

    p.legend(loc='best',labelspacing=0.1 )

    p.xlim(xmin=vmin, xmax=vmax)
    p.ylim(ymin=vmin, ymax=vmax)

    p.grid()
    p.title('Least Squares Circle')
    p.savefig('%s_residu%d.png' % (basename, 2 if residu2 else 1))

plot_all(residu2=False)
#plot_all(residu2=True )

p.show()
# vim: set et sts=4 sw=4:
