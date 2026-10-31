import os
import subprocess
import sys
from pathlib import Path

from ponte_visao_drone.filtro_stderr import eh_ruido

PACOTE = Path(__file__).resolve().parents[1]


def test_libjpeg_e_mediapipe_sao_ruido():
    assert eh_ruido("Corrupt JPEG data: premature end of data segment")
    assert eh_ruido("Corrupt JPEG data: 1200 extraneous bytes before marker 0xd9")
    assert eh_ruido("Premature end of JPEG file")
    assert eh_ruido("INFO: Created TensorFlow Lite XNNPACK delegate for CPU.")
    assert eh_ruido("WARNING: Logging before InitGoogle() is written to STDERR")
    assert eh_ruido("W0000 00:00:1793474050.697891    8371 inference_feedback_manager.cc:121] Feedback manager")
    assert eh_ruido("I0000 00:00:1793474050.697891    8371 gl_context.cc:369] GL version: 3.2")
    assert eh_ruido("QFontDatabase: Cannot find font directory /x/cv2/qt/fonts.")
    assert eh_ruido("Note that Qt no longer ships fonts. Deploy some (from https://dejavu-fonts.github.io/)")
    assert eh_ruido("Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland")


def test_logs_do_ros_e_erros_do_mediapipe_passam():
    assert not eh_ruido("[WARN] [1793474050.69] [piloto_gestos]: Desarmado: perdi a telemetria do celular.")
    assert not eh_ruido("[INFO] [1793474050.69] [piloto_gestos]: ARMADO")
    assert not eh_ruido("E0000 00:00:1793474050.697891    8371 calculator_graph.cc:887] INTERNAL: falhou")
    assert not eh_ruido("Traceback (most recent call last):")


def test_filtra_o_descritor_2_e_entrega_tudo_ao_restaurar():
    # Num processo à parte: o pytest também mexe no descritor 2.
    codigo = (
        "import os\n"
        "from ponte_visao_drone import filtro_stderr\n"
        "filtro_stderr.instalar()\n"
        "os.write(2, b'Corrupt JPEG data: premature end of data segment\\n')\n"
        "os.write(2, b'[WARN] [1.0] [piloto_gestos]: Desarmado\\n')\n"
        "os.write(2, b'sem quebra no fim')\n"
        "filtro_stderr.restaurar()\n"
        "os.write(2, b'\\ndepois\\n')\n"
    )
    ambiente = dict(os.environ, PYTHONPATH=str(PACOTE))
    saida = subprocess.run([sys.executable, "-c", codigo], env=ambiente, capture_output=True, timeout=10)
    assert saida.stderr.decode() == "[WARN] [1.0] [piloto_gestos]: Desarmado\nsem quebra no fim\ndepois\n"
