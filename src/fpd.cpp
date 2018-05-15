
#include<iostream>
#include <time.h>
#include <fstream>      // std::fstream


#include "MiddlewareInterface.h"
#include "fcontrol.h"
#include "IPlot.h"



using namespace std;

#define ROBOT "teo"
bool useRobot = 0;

int main()
{

    double dts = 0.01;

    MWI::Limb rightArm(ROBOT,"rightArm");

    if (useRobot)
    {
       // rightArm = MWI::Limb(ROBOT,"rightArm");
        if (rightArm.GetError()!=0)
        {
            std::cout << "MWI::Limb rightArm(\"rightArm\") not available. ERROR: " << rightArm.GetError() << std::endl;
            return -1;

        }
        rightArm.SetControlMode(1);
        rightArm.SetJointPositions(std::vector<double>{0,0,0,0,0,0});
        yarp::os::Time::delay(5);
        //rightArm.DefaultPosition();
        //yarp::os::Time::delay(5);
        rightArm.SetControlMode(2);



    }

    //instantiate object motor
    double ka=1;//10.09;//acceleration
    SystemBlock acc(
                std::vector<double> {ka},
                std::vector<double> {1}
                );
    //graph: acc.SetSaturation(-24,24);
    acc.SetSaturation(-10,10);
    //instantiate object motor
    SystemBlock modelVel(
                std::vector<double> {1,1},
                std::vector<double> {-1,+1},
//                std::vector<double> {0,dts*1},
//                std::vector<double> {-1,1}
//                std::vector<double> {Ts*Ts*ka,2*Ts*Ts*ka,Ts*Ts*ka},
//                std::vector<double> {Ts*Ts*ka+4,(2*Ts*Ts*ka-8),(Ts*Ts*ka+4)}
                dts/2);
    //TODO: Update <maxvel>10</maxvel> and <maxaccel>5</maxaccel> in openrave joints
    modelVel.SetSaturation(-24.4,24.4);
    //instantiate object encoder
    SystemBlock modelEncoder(
                std::vector<double> {dts,dts},
                std::vector<double> {-2,+2}
//                std::vector<double> {0,Ts*1},
//                std::vector<double> {-1,1}
                );


    //instantiate object control

    double N = 20;    // LPFfilter N
    //matlab fod ts=0.01
//    SystemBlock fod(
//                std::vector<double> {1.7963096, - 14.815094, + 35.195846, - 33.176969, + 11},
//                std::vector<double> {0.5187250, - 2.3022224, + 4.0225029, - 3.2386161, + 1},
//                1 //fod gain
//                );
    //scilab fod ts=0.01
    TimeSignal s_0_99(std::valarray<double>{//Mu=0.99
                                            95.498853693454038,-94.544661864875906,-0.47310848856560928,-0.15952833125659127,-0.080343435133966218,-0.048506626138845481,-0.032531616119190643,-0.023377331441217768,-0.017642059394224276,-0.013810140822483625,-0.011122224554798355,-0.0091634237495878939,-0.0076913711338489569,-0.0065567275885293442,-0.0056633796374972907,-0.0049471848863657641,-0.0043640030304880617,-0.0038826482165719156,-0.0034805782244046318,-0.0031411695656536516,-0.002851944926242897,-0.0026033906094514378,-0.0023881497026380823,-0.0022004604451741549,-0.0020357581666588329,-0.001890388518382875,-0.0017613977978444175,-0.0016463775519071433,-0.0015433479710844045,-0.0014506693866303364,-0.0013669743868059089,-0.0012911152373902221,-0.0012221227857693862,-0.0011591740680430007,-0.0011015665745100489,-0.0010486976529162778,-0.0010000479100850598,-0.00095516774796629983,-0.00091366637552628416,-0.00087520278889685957,-0.00083947832737072195,-0.00080623049731635629,-0.00077522782380730869,-0.00074626553803512803,-0.00071916194985509807,-0.00069375538288775785,-0.00066990157513715674,-0.00064747146576131379,-0.00062634930370158992,-0.00060643102589619247,-0.00058762286180784329,-0.00056984012915679491,-0.00055300619087443603,-0.00053705154986368603,-0.00052191306039007547,-0.00050753323963866493,-0.00049385966502098742,-0.00048084444518213141,-0.0004684437546041934,-0.00045661742335922125,-0.000445328574255933,-0.0004345433016749486,-0.00042423038646351548,-0.00041436104227980468
                      }, dts);

    TimeSignal s_1_00(std::valarray<double>{//Mu=1
                                            1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
                      }, dts);

    TimeSignal s_0_01(std::valarray<double>{//mu=0.01
                                            1.0314350837020543,-0.0261607675050918,-0.020868875039266536,-0.019120027439622732,-0.01824814432696114,-0.017725339249244118,-0.017376527896914597,-0.017126922683084377,-0.016939222012240487,-0.016792743152987129,-0.016675096248944365,-0.016578407342934642,-0.01649743303289504,-0.016428546733788636,-0.016369160120192721,-0.016317376375340292,-0.01627177368659196,-0.016231265279991886,-0.016195006186316557,-0.016162329497266766,-0.016132701775602807,-0.016105691225537518,-0.016080944557575923,-0.016058169897948494,-0.016037123977166282,-0.016017602397880585,-0.015999432151860393,-0.015982465802186215,-0.015966576913795154,-0.0159516564306472,-0.01593760977834555,-0.015924354528186514,-0.015911818499671049,-0.015899938208355491,-0.015888657587856264,-0.015877926931114429,-0.015867702008237165,-0.015857943327473555,-0.01584861551293049,-0.015839686778054536,-0.015831128478105427,-0.015822914728121737,-0.015815022075456348,-0.015807429217990214,-0.015800116760753088,-0.015793067004973661,-0.01578626376462083,-0.01577969220634089,-0.015773338709376951,-0.015767190742615803,-0.015761236756363685,-0.015755466086829782,-0.015749868871606659,-0.015744435974695603,-0.015739158919839964,-0.015734029831109882,-0.015729041379832016,-0.015724186737086258,-0.01571945953109774,-0.015714853808944135,-0.015710364002075268,-0.015705984895208323,-0.01570171159821818,-0.01569753952069022
                      }, dts);

    TimeSignal s_0_263(std::valarray<double>{//mu=0.263
                                             3.3410117042574567,-0.89924887760259908,-0.34153813719616088,-0.20445255637366488,-0.14486928643608676,-0.11221192312437531,-0.091838522921276811,-0.078024026621223055,-0.068094067707615155,-0.06064099802776092,-0.054857216366047137,-0.05024807530285081,-0.046494626617538427,-0.043382491606593299,-0.040762551666362029,-0.038528051777334246,-0.036600666466076293,-0.034921699164479514,-0.033446343645971613,-0.032139837024157812,-0.030974817396991223,-0.029929469504662142,-0.028986198252191377,-0.028130663408815312,-0.027351066185274768,-0.026637614515390193,-0.02598211712327568,-0.025377671734350026,-0.02481842301122969,-0.024299372753184952,-0.023816229707170296,-0.023365289710724903,-0.02294333928315706,-0.02254757750496611,-0.022175552279450311,-0.021825107992547568,-0.021494342271825825,-0.021181570058930038,-0.020885293598058845,-0.020604177239085433,-0.020337026181428592,-0.020082768460877767,-0.019840439618835564,-0.01960916960113742,-0.019388171518631557,-0.01917673196922412,-0.018974202675025857,-0.018779993231536066,-0.018593564800738701,-0.018414424608340196,-0.018242121128450483,-0.018076239857924926,-0.017916399598101939,-0.017762249174496886,-0.017613464535632166,-0.017469746181010024,-0.017330816875612561,-0.017196419614480971,-0.017066315806119593,-0.016940283647836955,-0.016818116669845028,-0.016699622428063547,-0.016584621328256388,-0.016472945566393841
                       }, dts);

    TimeSignal s_0_669(std::valarray<double>{//mu=0.669
                                             21.765978091289533,-14.579824268822982,-2.4219276017199745,-1.0803629378313908,-0.63385474130541331,-0.42561431702216701,-0.30994384397341768,-0.23832579322527719,-0.19055855379120321,-0.1569205589047018,-0.13222915581191841,-0.11350142023186792,-0.098914972729732722,-0.087301895915795846,-0.077883727487394971,-0.070124275099404471,-0.063643779611711887,-0.058166751397993766,-0.053489133137779865,-0.049456988327040291,-0.045952295982679081,-0.042883256436992714,-0.040177535706052175,-0.037777468616930904,-0.035636594795416546,-0.03371711865158232,-0.031988020824529492,-0.030423636052024857,-0.029002569718478609,-0.027706863521680718,-0.026521346578174579,-0.025433126094733513,-0.024431184160347286,-0.023506055998937292,-0.022649571310445316,-0.021854644878278706,-0.021115105949447988,-0.020425558352194256,-0.019781265148946597,-0.019178053000927897,-0.018612232466100331,-0.018080531250944457,-0.017580038051480437,-0.017108155095559754,-0.016662557870514003,-0.016241160812000201,-0.015842087960819113,-0.01546364777741218,-0.015104311450183067,-0.014762694151180922,-0.014437538787379134,-0.01412770187256604,-0.013832141207392457,-0.013549905106122764,-0.013280122950702022,-0.013021996887206816,-0.012774794508467928,-0.012537842390297996,-0.012310520368634632,-0.012092256461431698,-0.011882522352992503,-0.011680829370173359,-0.011486724889708144,-0.011299789124290166
                       }, dts);

    TimeSignal s_0_66(std::valarray<double>{//mu=0.66
                                            20.881621650144087,-13.800519155459961,-2.355186076968744,-1.0579043720415218,-0.62321202803194442,-0.4196973740418134,-0.30634494186688532,-0.23601512077742348,-0.18902699865908837,-0.15588989988553723,-0.13153571830253266,-0.11304330987989733,-0.098625846993234373,-0.087136958523596819,-0.077811805854210617,-0.070123120860831459,-0.063697167864780552,-0.058262628535774463,-0.053618399217064572,-0.049612675994648053,-0.046129004316695477,-0.043076752770876206,-0.040384470617157282,-0.037995168667741698,-0.035862909652509864,-0.033950306849279778,-0.03222666339517069,-0.03066657051973281,-0.029248839155822368,-0.027955676873830368,-0.026772047502948586,-0.025685168300716231,-0.024684111747429682,-0.023759487681841662,-0.022903187679490955,-0.022108178052963612,-0.021368331129870854,-0.020678286885253389,-0.020033338810689222,-0.019429339260810141,-0.018862620548147357,-0.018329928844895621,-0.017828368556559317,-0.017355355302629737,-0.016908576006556077,-0.016485954885276828,-0.016085624356445411,-0.015705900062256608,-0.01534525935334638,-0.01500232269217705,-0.014675837529009361,-0.014364664279320899,-0.014067764093415036,-0.013784188159405394,-0.013513068322309439,-0.013253608836107454,-0.013005079094036299,-0.012766807205739771,-0.012538174309671943,-0.012318609525341037,-0.012107585463920163,-0.011904614227150626,-0.01170924383436908,-0.011521055025679563
                      }, dts);

    TimeSignal s_0_47(std::valarray<double>{//mu=0.47
                                            8.6947873016269988,-4.1082107513579551,-1.0993055262505922,-0.56760426191507474,-0.36413659187300462,-0.2611114609083508,-0.20044199869109541,-0.16113383068373266,-0.13392309185369622,-0.11414688993034885,-0.099226578450616165,-0.087631322443893406,-0.07840047462369884,-0.070903661000931623,-0.064711740987174207,-0.059523426512104564,-0.055121574369670481,-0.051346113041428554,-0.048076700701088812,-0.045221288519787586,-0.042708383651057773,-0.040481695874086482,-0.03849635854141642,-0.036716212529966957,-0.035111822340475066,-0.033659005583594824,-0.03233772835643034,-0.031131265282996382,-0.030025553612905334,-0.029008691394002237,-0.028070543846730755,-0.027202431869023078,-0.026396883501064394,-0.025647434099607569,-0.024948464521428182,-0.024295069204815833,-0.023682947946382121,-0.023108316590190775,-0.022567832912309391,-0.022058534791026638,-0.021577788369068998,-0.021123244387920313,-0.020692801241346426,-0.020284573581316898,-0.019896865534163873,-0.019528147762039594,-0.019177037745628751,-0.018842282776437992,-0.018522745237305779,-0.018217389822549501,-0.017925272408231399,-0.017645530331047307,-0.017377373873711473,-0.017120078786961868,-0.016872979704958019,-0.016635464332885513,-0.016406968303905412,-0.016186970617853659,-0.015974989586886205,-0.015770579223991638,-0.015573326019333367,-0.015382846057022038,-0.015198782431384308,-0.01502080292729405
                      }, dts);


    //TODAS LAMBDA MAL
    TimeSignal is_0_00(std::valarray<double>{//lambda=0

                                             1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
                       }, dts);
    TimeSignal is_0_854(std::valarray<double>{//lambda=0.854
                                              //0.86499999999999999,0.80661249999999995,0.77031493749999991,0.74431680835937497,0.72422025453367178,0.70792529880666422,0.69427245375824997,0.68255660610107949,0.67231825700956327,0.66324196053993423,0.65510217284239869,0.64773227339792172,0.64100582286648178,0.63482469528884067,0.62911127303124115,0.62380314666504011,0.61884941579446484,0.61420804517600636,0.60984393538133475,0.60572748881751071,0.60183352638939813,0.59814045702291774,0.59462963260126145,0.59128484091787936,0.5880919027769228,0.58503834866635029,0.58211315692301857,0.57930653991642544,0.57660976809267661,0.5740150241362596,0.57151528128921458,0.56910420119627569,0.56677604764592726,0.56452561333909779,0.56234815740193267,0.56023935181167539,0.55819523525776793,0.55621217323777328,0.55428682340733482,0.55241610537833508,0.55059717429965027,0.54882739766797284,0.54710433490785237,0.54542571933483963,0.54378944217683511,0.54219353837914219,0.54063617396039365,0.53911563472113,0.5376303161356738,0.5361787142821075,0.53475941768547841,0.53337109996648724,0.53201251320242171,0.53068248191941569,0.52937989764561344,0.52810371396378919,0.52685294200966448,0.52562664636877987,0.52442394133047843,0.52324398746248491,0.52208598847383847,0.52094918833764547,0.51983286864835054,0.51873634619104547
                                              0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958,0.55171687002097958
                        }, dts);
    TimeSignal is_0_414(std::valarray<double>{//lambda=0.414
                                              1,0.41399999999999992,0.2926979999999999,0.23552432399999992,0.20102001053399993,0.17746046529941514,0.16012849318850558,0.14672345075872498,0.13597595799064838,0.1271224122814795,0.1196730389217848,0.11329772975740608,0.10776502395425276,0.10290730825908413,0.09859990235623961,0.094747932837522517,0.091277789797348255,0.088131390690216124,0.085262224304412423,0.082632557807444759,0.080211423863686626,0.077973143178728513,0.075896222183149648,0.073962518435352884,0.072156600276889679,0.070465249566399379,0.068877071249248994,0.067382183702876408,0.065971970858237639,0.064638882067791864,0.063376269238067662,0.062178253309889991,0.061039614046152636,0.059955698475514893,0.058922344378260431,0.057935815983812984,0.056992749645854247,0.056090107719030717,0.055225139215784612,0.054395346098337176,0.053598454277996532,0.052832388565632972,0.052095250953741047,0.051385301719813321,0.050700942928726719,0.050040703983032633,0.049403228927944437,0.048787265265140704,0.048191654068362108,0.047615321225830266,0.047057269661063536,0.046516572405742293,0.045992366416708348,0.045483847044629273,0.044990263074848665,0.044510912271905734,0.044045137368489008,0.04359232244747261,0.043151889672399871,0.042723296327518069,0.042306032133385976,0.041899616808301315,0.041503597849435754,0.041117548510709256,0.040741065957158075
                        }, dts);
    TimeSignal is_0_99(std::valarray<double>{//lambda=0.99
                                             //0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434,0.95926411932526434
                                             1,0.98999999999999999,0.98504999999999998,0.98176649999999999,0.97931208375000001,0.97735345958249997,0.9757245371498624,0.97433064495393407,0.9731127316477417,0.97203149527924426,0.97105946378396502,0.97017668245325239,0.96936820188454131,0.96862253403693788,0.96793066079834011,0.9672853736911412,0.96668082033258429,0.96611218455591807,0.9655754555644982,0.96506725795630632,0.96458472432732822,0.96412539826812471,0.96368715945073002,0.96326816503357748,0.96286680329814689,0.96248165657682772,0.96211147132429808,0.96175513374232613,0.96141164976598958,0.96108012850744962,0.96075976846461386,0.96044984595865746,0.96014970538179534,0.95985875092561912,0.95957643952828808,0.95930227483127994,0.9590358019771601,0.95877660311176083,0.95852429347936297,0.95827851801949648,0.9580389483899916,0.95780528035379886,0.95757723147752416,0.95735453909811075,0.95713695852104297,0.95692426141914944,0.95671623440579745,0.95651267776017923,0.95631340428564582,0.95611823828477116,0.95592701463711427,0.95573957796757758,0.95555578189489154,0.95537548835113784,0.95519856696440619,0.95502489449768535,0.95485435433795363,0.95468683603017501,0.95452223485154919,0.95436045142191328,0.95420139134667636,0.95404496488907853,0.95389108666893518,0.95373967538533699,0.9535906535610581
                        }, dts);
    TimeSignal is_0_792(std::valarray<double>{//lambda=0.792
                                              1,0.79200000000000004,0.70963200000000004,0.66043084800000007,0.62608844390400009,0.60004316463759366,0.57924166826349044,0.56202991583508954,0.54741713802337721,0.53476571972239251,0.52364259275216674,0.51374098736194396,0.50483614358100359,0.49675876528370755,0.48937834934234958,0.48259230289813565,0.47631860296045986,0.47049070475953192,0.46505392328231066,0.45996280664848327,0.45517919345933905,0.45067075192412276,0.44640986481502193,0.44237276690712957,0.4385388695939344,0.43489022619891288,0.43141110438932156,0.42808764106661867,0.42490756144726666,0.42185994859274834,0.41893505294917194,0.41612413388422265,0.41341932701397521,0.41081353246794777,0.40830032026932034,0.40587384979457697,0.40352880088465276,0.40126031465265255,0.39906394240402754,0.39693560137787276,0.39487153625070781,0.39286828553021641,0.39092265211616201,0.3890316774268559,0.38719261858811077,0.38540292826219241,0.38366023676048511,0.38196233613822594,0.38030716601496029,0.37869280090208046,0.37711743885032778,0.37557939125658524,0.3740770736915589,0.37260899762876937,0.37117376297123633,0.3697700512858178,0.36839661966675619,0.36705229515990206,0.36573596968760447,0.36444659542158714,0.36318318055745896,0.36194478544998437,0.36073051907299086,0.35953953577192449,0.35837103228066575
                        }, dts);


    double kp;
    double ki;
    double kd;

    //W=1-2

//    //ABC (3)//HS(11)
//    kp=0.009;
//    ki=0;
//    kd=0.996;
//    FSystemBlock simFs(s_0_99);
//    FSystemBlock simF1s(is_0_00);
//    FSystemBlock teoFs(s_0_99);
//    FSystemBlock teoF1s(is_0_00);

//    //PSO (4) 6?
//    kp=0.995;
//    ki=0.09;
//    kd=0.006;
//    FSystemBlock simFs(s_0_01);
//    FSystemBlock simF1s(is_0_99);
//    FSystemBlock teoFs(s_0_01);
//    FSystemBlock teoF1s(is_0_99);



//    //HS (8)
//    kp=0.996;
//    ki=0.094;
//    kd=0.01;
//    FSystemBlock simFs(s_0_263);
//    FSystemBlock simF1s(is_0_792);//MAL is
//    FSystemBlock teoFs(s_0_263);
//    FSystemBlock teoF1s(is_0_792);

//    //isow1
//    kp=0.541;
//    ki=0.0;
//    kd=0.541;
//    FSystemBlock simFs(s_0_47);
//    FSystemBlock simF1s(is_0_00);
//    FSystemBlock teoFs(s_0_47);
//    FSystemBlock teoF1s(is_0_00);

//    //cmon
//    kp=1.14;
//    ki=0.0;
//    kd=0.768;
//    FSystemBlock simFs(s_0_669);
//    FSystemBlock simF1s(is_0_00);
//    FSystemBlock teoFs(s_0_669);
//    FSystemBlock teoF1s(is_0_00);

    //isow2
    kp=1.17;
    ki=0.0;
    kd=0.64;
    FSystemBlock simFs(s_0_66);
    FSystemBlock simF1s(is_0_00);
    FSystemBlock teoFs(s_0_66);
    FSystemBlock teoF1s(is_0_00);

    //dts=0.01 //w=10 pm60
    FactorSystemBlock control(vector<double>{-0.9951 ,  -0.9741 ,  -0.9120 ,  -0.7332},
                           vector<double>{-0.9950 ,  -0.9729 ,  -0.8973  , -0.0336},
                           266.04 );


    double signal;
    double modelSignal;
    double jointPos, jointLastPos, jointVel;


    //time_t t;
    modelEncoder.Reset(60);
    double target = 30;
    double error, modelError;
    int jointNumber = 3;

    IPlot pt(dts),vt(dts),at(dts),con(dts);
    IPlot ptTeo(dts),vtTeo(dts),atTeo(dts),conTeo(dts);


    //control loop
    long loops = 20/dts;
    //rightArm.SetJointPos(jointNumber,target);

    for (ulong i=0; i<loops; i++)
    {

        //MODEL BLOCK DIAGRAM
        modelError = target-modelEncoder.GetState();

        //signal out from controller
//        modelSignal = kd*(modelError > simFs);
//        modelSignal += ki*(modelError > simF1s);
//        //modelSignal *= kd;
//        modelSignal += modelError*kp;
        modelSignal= modelError > control;
        //next lines simulates model setjointVel
        if (  modelVel.GetState() > modelSignal )
        {
            //constant deceleration of model
            -15 > acc > modelVel  >  modelEncoder;

        }
        else
        {
            //constant acceleration of model
            15 > acc > modelVel  >  modelEncoder;

        }


        //ROBOT BLOCK DIAGRAM
        if (useRobot)
        {

            jointVel = (jointPos-jointLastPos)/dts;
            vtTeo.pushBack(jointVel);

            jointLastPos = jointPos;
//            jointPos = linFilter(rightArm.GetJoint(jointNumber),i*Ts);

            jointPos = rightArm.GetJoint(jointNumber);

            error=target-jointPos;
            //error = error/(Ts*Ts);
            signal = kd*(error > teoFs);
            signal += ki*(error > teoF1s);
            //signal *= kd;
            signal += error*kp;

            if (fabs(jointVel)>14.4)
            {
            //signal = signal*15/24.4; //correct signal as 15 value for vel equals to 24.4 deg/sec
            }

            rightArm.SetJointVel(jointNumber,1*signal);
            yarp::os::Time::delay(dts);
            //signal = signal*24.4/15;

            //plot data store
            ptTeo.pushBack(jointPos);
            vtTeo.pushBack(jointVel);
            conTeo.pushBack(signal);

            std::cout << i*dts

                      << " , signal: " << signal
                      << " , jointVel: " << jointVel
                      << " , jointPos: " << jointPos
                      << std::endl;

        }

        //plot data store
        pt.pushBack(modelEncoder.GetState());
        vt.pushBack(modelVel.GetState());
        at.pushBack(acc.GetState());
        con.pushBack(modelSignal);


        std::cout << i*dts

                  << " , modelSignal: " << modelSignal
                  << " , modelVel: " << modelVel.GetState()
                  << " , modelPos: " << modelEncoder.GetState()
                  << std::endl;

    }

    pt.Plot();
    pt.Save("/home/buyus/Escritorio/ptSim.csv");
    con.Save("/home/buyus/Escritorio/conSim.csv");

    if (useRobot)
    {
        ptTeo.Plot();
        ptTeo.Save("/home/buyus/Escritorio/ptTeo.csv");
        conTeo.Save("/home/buyus/Escritorio/conTeo.csv");
        //vtTeo.Plot();
        //vtTeo.Save("vtTeo.txt");


        rightArm.SetJointVel(jointNumber,0.);

    }


    return 0;

}
