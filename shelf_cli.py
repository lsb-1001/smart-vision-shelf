import argparse
import json
from pathlib import Path

from shelf_core import Registry, ArrivalVerifier

DEFAULT = Path(__file__).with_name('locations.json')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['list','lookup','demo'])
    parser.add_argument('location', nargs='?', default='B')
    parser.add_argument('--registry', default=str(DEFAULT))
    args = parser.parse_args()
    registry = Registry.load(args.registry)
    if args.command == 'list':
        for key in registry.by_key:
            print(json.dumps(registry.lookup(key),ensure_ascii=False))
    elif args.command == 'lookup':
        print(json.dumps(registry.lookup(args.location),indent=2,ensure_ascii=False))
    else:
        print('SIMULATED OBSERVATIONS. No camera detection or robot movement.')
        target = registry.get(args.location)
        for name, ids in [('correct',[target['marker_id']]),
                          ('wrong',[1 if target['marker_id'] != 1 else 2]),
                          ('unknown',[49]),('multiple',[1,2]),('not_visible',[])]:
            verifier = ArrivalVerifier(registry)
            verifier.begin(args.location,'demo-'+name,0,navigation_arrived=True)
            for step in range(5):
                result = verifier.observe(ids,0.1+step*0.2)
            print(name, '=>', result['state'], 'observed=',ids)
        verifier = ArrivalVerifier(registry)
        verifier.begin(args.location,'demo-camera-loss',0,navigation_arrived=True)
        print('camera_loss =>',verifier.tick(2)['state'])


if __name__ == '__main__':
    main()
