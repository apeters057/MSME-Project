import sys
sys.path.append('/home/apeters/project/env/src/robot-hat')
from robot_hat import Music, Buzzer

m = Music()        # create a music object
buzzer = Buzzer("P0")
m.tempo(120)          # set current tempo to 120 beat per minute

# play middle C, D, E, F ,G, A, B every 1 beat.
buzzer.play(m.note("Middle C"), m.beat(1))
buzzer.play(m.note("Middle D"), m.beat(1))
buzzer.play(m.note("Middle E"), m.beat(1))
buzzer.play(m.note("Middle F"), m.beat(1))
buzzer.play(m.note("Middle G"), m.beat(1))
buzzer.play(m.note("Middle A"), m.beat(1))
buzzer.play(m.note("Middle B"), m.beat(1))

song = './music/test.wav'

m.music_set_volume(80)
print('Music duration',m.sound_length(file_name))
m.sound_play(song)